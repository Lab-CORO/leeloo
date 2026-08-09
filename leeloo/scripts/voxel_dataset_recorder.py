#!/usr/bin/env python3
"""Enregistre le topic voxel sparse de curobo dans un dataset HDF5.

Source : ``curobo_msgs/msg/SparseVoxelGrid`` publié par le planificateur
(``/curobo_trajectory_planner/voxel_grid_sparse`` dans le bringup Leeloo). Le
message ne transporte que les indices linéaires C-order des voxels OCCUPÉS :

    linear = x * size_y * size_z + y * size_z + z

Convention du dataset produit (identique à ``curobo_msgs/msg/GpuVoxelTensor``,
donc directement consommable par le U-Net sans étape d'inversion) :

    0 = voxel occupé
    1 = voxel libre

Sortie : UN fichier .h5 par session, avec UN dataset par frame sous le groupe
``voxel_grid`` — ``voxel_grid/0``, ``voxel_grid/1``, ... ``voxel_grid/n`` — de
forme ``(size_x, size_y, size_z)`` en ``uint8``, compressé. La géométrie de la
grille (origin, resolution, dimensions, frame_id) est écrite en attributs du
fichier ; une frame dont la géométrie diffère de la première est ignorée (les
datasets du fichier resteraient hétérogènes sinon).
"""

import os
from datetime import datetime

import h5py
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from curobo_msgs.msg import SparseVoxelGrid


class VoxelDatasetRecorder(Node):
    """Souscrit au topic voxel sparse et accumule les grilles denses en HDF5."""

    def __init__(self):
        super().__init__('voxel_dataset_recorder')

        self.declare_parameter('topic', '/curobo_trajectory_planner/voxel_grid_sparse')
        self.declare_parameter('output_dir', os.path.expanduser('~/voxel_datasets'))
        # Vide => nom auto voxels_<YYYYmmdd_HHMMSS>.h5 (jamais d'écrasement).
        self.declare_parameter('filename', '')
        # 0 = illimité ; sinon le node s'arrête proprement une fois atteint.
        self.declare_parameter('max_frames', 0)
        # Ignore une frame strictement identique à la précédente : sans ça une
        # scène statique publiée à 7 Hz remplit le dataset de doublons.
        self.declare_parameter('deduplicate', True)
        self.declare_parameter('compression', 'gzip')  # 'gzip', 'lzf' ou '' (aucune)
        self.declare_parameter('compression_level', 4)
        self.declare_parameter('flush_every', 20)
        # Coupe le node quand max_frames est atteint (utile en launch/script).
        self.declare_parameter('shutdown_when_done', True)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.max_frames = self.get_parameter('max_frames').get_parameter_value().integer_value
        self.deduplicate = self.get_parameter('deduplicate').get_parameter_value().bool_value
        self.flush_every = self.get_parameter('flush_every').get_parameter_value().integer_value
        self.shutdown_when_done = self.get_parameter(
            'shutdown_when_done').get_parameter_value().bool_value

        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        filename = self.get_parameter('filename').get_parameter_value().string_value
        if not filename:
            filename = f'voxels_{datetime.now().strftime("%Y%m%d_%H%M%S")}.h5'
        os.makedirs(output_dir, exist_ok=True)
        self.output_path = os.path.join(output_dir, filename)

        self.h5_file = h5py.File(self.output_path, 'w')
        self.group = None         # groupe 'voxel_grid', créé à la première frame
        self.grid_shape = None    # (size_x, size_y, size_z) figée par la 1re frame
        self.frame_count = 0
        self.skipped_geometry = 0
        self.skipped_duplicate = 0
        self.last_signature = None
        self.done = False

        self.subscription = self.create_subscription(
            SparseVoxelGrid, self.topic, self.on_voxel_grid, 10)

        self.get_logger().info(
            f'Enregistrement de {self.topic} vers {self.output_path} '
            f'(max_frames={self.max_frames or "illimité"}, dedup={self.deduplicate})')

    # ------------------------------------------------------------------ écriture

    def _write_frame(self, index, dense):
        """Écrit une frame dans son propre dataset ``voxel_grid/<index>``."""
        kwargs = {
            # Un chunk = une frame entière : c'est l'unité de lecture d'un
            # DataLoader, qui charge un voxel_grid/<i> complet à la fois.
            'chunks': dense.shape,
            'dtype': 'uint8',
        }
        compression = self.get_parameter('compression').get_parameter_value().string_value
        if compression:
            kwargs['compression'] = compression
            if compression == 'gzip':
                kwargs['compression_opts'] = self.get_parameter(
                    'compression_level').get_parameter_value().integer_value
        self.group.create_dataset(str(index), data=dense, **kwargs)

    def _init_file(self, msg):
        size = (int(msg.size_x), int(msg.size_y), int(msg.size_z))
        self.grid_shape = size
        self.group = self.h5_file.create_group('voxel_grid')

        # Géométrie de la grille : nécessaire pour repositionner le volume dans
        # le repère base. Figée par la première frame, cf. _geometry_matches().
        attrs = self.h5_file.attrs
        attrs['origin'] = np.array(
            [msg.origin.x, msg.origin.y, msg.origin.z], dtype=np.float64)
        attrs['resolution'] = float(msg.resolution)
        attrs['size_x'] = size[0]
        attrs['size_y'] = size[1]
        attrs['size_z'] = size[2]
        attrs['frame_id'] = msg.header.frame_id
        attrs['topic'] = self.topic
        attrs['occupancy_convention'] = '0 = occupied, 1 = free'
        attrs['index_order'] = 'C-order (x, y, z)'
        attrs['created'] = datetime.now().isoformat()

        self.get_logger().info(
            f'Groupe créé : voxel_grid/<i> ({size[0]}, {size[1]}, {size[2]}) uint8, '
            f'origin=({msg.origin.x:.3f}, {msg.origin.y:.3f}, {msg.origin.z:.3f}), '
            f'resolution={msg.resolution:.4f} m, frame_id={msg.header.frame_id}')

    def _geometry_matches(self, msg):
        """La bbox de curobo peut bouger (mapper_grid_center / voxel_size).

        La géométrie est décrite une seule fois, en attributs du fichier : une
        frame de géométrie différente y serait silencieusement mal interprétée
        à la relecture. On la rejette plutôt que de mélanger deux grilles
        incompatibles dans le même fichier.
        """
        attrs = self.h5_file.attrs
        return (
            (int(msg.size_x), int(msg.size_y), int(msg.size_z)) == self.grid_shape
            and np.allclose(
                attrs['origin'],
                [msg.origin.x, msg.origin.y, msg.origin.z], atol=1e-6)
            and abs(float(attrs['resolution']) - float(msg.resolution)) < 1e-9
        )

    # ------------------------------------------------------------------ callback

    def on_voxel_grid(self, msg):
        if self.done:
            return

        if self.group is None:
            self._init_file(msg)
        elif not self._geometry_matches(msg):
            self.skipped_geometry += 1
            self.get_logger().warn(
                f'Géométrie de grille différente de celle du fichier '
                f'({msg.size_x}x{msg.size_y}x{msg.size_z} @ {msg.resolution:.4f} m) '
                f'- frame ignorée ({self.skipped_geometry} au total). Relancer le '
                f'node pour démarrer un nouveau fichier.',
                throttle_duration_sec=5.0)
            return

        indices = np.asarray(msg.occupied_indices, dtype=np.int64)

        if self.deduplicate:
            # Les indices caractérisent entièrement la grille (géométrie déjà
            # validée) : bien moins cher à hasher que le volume dense.
            signature = hash(indices.tobytes())
            if signature == self.last_signature:
                self.skipped_duplicate += 1
                return
            self.last_signature = signature

        size_x, size_y, size_z = self.grid_shape
        n_voxels = size_x * size_y * size_z

        # 1 = libre partout, puis 0 sur les voxels occupés.
        dense = np.ones(n_voxels, dtype=np.uint8)
        if indices.size:
            valid = (indices >= 0) & (indices < n_voxels)
            if not valid.all():
                self.get_logger().warn(
                    f'{int((~valid).sum())} indice(s) hors grille ignoré(s)',
                    throttle_duration_sec=5.0)
                indices = indices[valid]
            dense[indices] = 0

        self._write_frame(self.frame_count, dense.reshape(size_x, size_y, size_z))
        self.frame_count += 1

        if self.flush_every > 0 and self.frame_count % self.flush_every == 0:
            self.h5_file.flush()
            self.get_logger().info(
                f'{self.frame_count} frames enregistrées '
                f'({self.skipped_duplicate} doublons ignorés)')

        if self.max_frames > 0 and self.frame_count >= self.max_frames:
            self.get_logger().info(f'max_frames ({self.max_frames}) atteint.')
            self.done = True
            if self.shutdown_when_done:
                self.close()
                rclpy.shutdown()

    # ------------------------------------------------------------------ fermeture

    def close(self):
        if self.h5_file is None:
            return
        if self.group is not None:
            # Les frames sont 0..num_frames-1 sous voxel_grid/ : le lecteur n'a
            # pas à énumérer/trier les clés du groupe pour connaître la plage.
            self.h5_file.attrs['num_frames'] = self.frame_count
        self.h5_file.flush()
        self.h5_file.close()
        self.h5_file = None
        self.get_logger().info(
            f'{self.output_path} fermé : {self.frame_count} frames '
            f'({self.skipped_duplicate} doublons, {self.skipped_geometry} '
            f'géométries incompatibles ignorés)')


def main(args=None):
    rclpy.init(args=args)
    node = VoxelDatasetRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C ou SIGTERM (arrêt d'un ros2 launch) : sortie normale, le
        # fichier est refermé par le finally ci-dessous.
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

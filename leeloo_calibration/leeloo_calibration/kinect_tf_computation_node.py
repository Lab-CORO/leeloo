#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
import tf_transformations
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import ros2_numpy
import numpy as np
from numpy.linalg import inv
class kinectTFComputationNode(Node):
    def __init__(self):
        super().__init__('kinect_tf_computation_node')
 
        # Création du broadcaster pour publier la nouvelle transformation
        self.tf_broadcaster = TransformBroadcaster(self)
 
        # Buffer et listener pour recevoir les transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        # Timer pour exécuter périodiquement le calcul
        self.timer = self.create_timer(0.1, self.timer_callback)  # Exécution toutes les 0.1 secondes
 
        self.declare_parameter('calibration_result_file',
            '/home/ros2_ws/src/leeloo_calibration/config/kinect_hand_eye_result.yaml')

        # Résultat de calibration hand-eye (tracking_base_frame=rgb_camera_link) :
        # = base_link → rgb_camera_link  (_base_T_rgb_)
        # Chargé depuis le fichier YAML généré par auto_calibration_server_v2.
        #
        # ATTENTION : les anciens résultats ci-dessous utilisaient tracking_base_frame=camera_base
        # → ils représentaient _base_T_camera_base_ (PAS _base_T_rgb_), d'où la rotation π et l'offset.
        #
        # Ancien résultat (tracking_base_frame=camera_base, NE PAS UTILISER) :
        # tx, ty, tz, qx, qy, qz, qw: [0.0948, -0.3107, 1.1214, 0.2943, -0.0057, 0.4459, 0.8453]
        # as euler: rpy: -0.2903, -0.9116, -2.5247  ← rotation π absorbée = BUG
        self.base_link_to_rgb = TransformStamped()
        self.base_link_to_rgb.header.frame_id = 'base_link'
        self.base_link_to_rgb.child_frame_id = 'rgb_camera_link'
  
        self._load_calibration()

        self.create_service(Trigger, '~/reload_calibration', self._reload_calibration_cb)
        self.get_logger().info('[INIT] Service ~/reload_calibration disponible.')

    def _reload_calibration_cb(self, request, response):
        """Recharge le fichier YAML de calibration à chaud."""
        self._load_calibration()
        response.success = True
        response.message = 'Calibration rechargée.'
        return response

    def _load_calibration(self):
        """Charge le transform de calibration depuis le fichier YAML."""
        filepath = self.get_parameter('calibration_result_file').value
        if not os.path.isfile(filepath):
            self.get_logger().warn(
                f'Fichier de calibration introuvable : {filepath}. '
                'Le transform base_link→rgb_camera_link ne sera pas publié.')
            return
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        t = data['transform']
        self.base_link_to_rgb.transform.translation.x = t['translation']['x']
        self.base_link_to_rgb.transform.translation.y = t['translation']['y']
        self.base_link_to_rgb.transform.translation.z = t['translation']['z']
        self.base_link_to_rgb.transform.rotation.x = t['rotation']['x']
        self.base_link_to_rgb.transform.rotation.y = t['rotation']['y']
        self.base_link_to_rgb.transform.rotation.z = t['rotation']['z']
        self.base_link_to_rgb.transform.rotation.w = t['rotation']['w']
        self.get_logger().info(
            f'Calibration chargée depuis {filepath} : '
            f'tx={t["translation"]["x"]:.4f} ty={t["translation"]["y"]:.4f} tz={t["translation"]["z"]:.4f} '
            f'qx={t["rotation"]["x"]:.4f} qy={t["rotation"]["y"]:.4f} '
            f'qz={t["rotation"]["z"]:.4f} qw={t["rotation"]["w"]:.4f}')

    def timer_callback(self):
        try:
            # Récupération de la transformation de calibration (camera_base <- rgb_camera_link)
            rgb_to_cam = self.tf_buffer.lookup_transform('camera_base', 'rgb_camera_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'Could not get calibration transform: {e}')
            return
 
        # Calcul de la transformation entre camera_link et camera_color_optical_frame
        base_link_to_camera_base_tf = self.compute_base_link_to_camera_base(rgb_to_cam, self.base_link_to_rgb)
 
        # Publication de la transformation calculée
        self.tf_broadcaster.sendTransform(base_link_to_camera_base_tf)
 
    def compute_base_link_to_camera_base(self, rgb_to_cam, base_link_to_rgb):
        """
        Calcule la transformation entre base_link et camera_base.
 
        Paramètres:
        - rgb_to_cam: TransformStamped représentant la transformation de camera_base vers rgb_camera_link.
        - base_link_to_rgb: TransformStamped représentant la transformation de base_link vers rgb_camera_link .
 
        Retourne:
        - TransformStamped représentant la transformation de base_link vers camera_base.
        """
 
        # Conversion de rgb_to_cam en matrice
        trans_calib = rgb_to_cam.transform.translation
        rot_calib = rgb_to_cam.transform.rotation
        T_rgb_camera_link_to_camera_base = ros2_numpy.numpify(rgb_to_cam.transform ) 
 
        # Conversion de base_link_to_rgb en matrice
        trans_cam = base_link_to_rgb.transform.translation
        rot_cam = base_link_to_rgb.transform.rotation
        T_base_link_to_rgb = ros2_numpy.numpify(base_link_to_rgb.transform) 
 
        # Calcul de T_base_link_camera_base 
        T_base_link_camera_base = np.dot((T_base_link_to_rgb), inv(T_rgb_camera_link_to_camera_base))#  @   

        # Extraction de la translation et rotation
        translation = tf_transformations.translation_from_matrix(T_base_link_camera_base)
        rotation = tf_transformations.quaternion_from_matrix(T_base_link_camera_base)
 
        # Création du TransformStamped résultat
        result_tf = TransformStamped()
        result_tf.header.stamp = self.get_clock().now().to_msg()
        result_tf.header.frame_id = 'world'
        result_tf.child_frame_id = 'camera_base'
        result_tf.transform.translation.x = translation[0]
        result_tf.transform.translation.y = translation[1]
        result_tf.transform.translation.z = translation[2]
        result_tf.transform.rotation.x = rotation[0]
        result_tf.transform.rotation.y = rotation[1]
        result_tf.transform.rotation.z = rotation[2]
        result_tf.transform.rotation.w = rotation[3]
 
        return result_tf
 
def main(args=None):
    rclpy.init(args=args)
 
    node = kinectTFComputationNode()
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
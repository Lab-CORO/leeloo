#!/usr/bin/env python3
"""
Recalibration offline à partir de samples sauvegardés.

Charge un fichier YAML généré par auto_calibration_server en mode debug_capture,
filtre les points par ambiguity_ratio, et recalcule la calibration via cv2.calibrateHandEye.

Utilise : python3 recalibrate_from_samples.py --input FILE --threshold RATIO --method METHOD --output OUT

Aucune dépendance ROS — seulement numpy, cv2, transforms3d, yaml.
"""

import argparse
import sys
import yaml
import numpy as np
import cv2
import transforms3d


DEFAULT_INPUT = '/home/ros2_ws/src/leeloo_calibration/config/calibration_debug_samples.yaml'
DEFAULT_THRESHOLD = 2.0
DEFAULT_METHOD = 'Tsai-Lenz'
MIN_SAMPLES = 3  # OpenCV demande au minimum 3 points pour calibrateHandEye

AVAILABLE_METHODS = {
    'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
    'Park': cv2.CALIB_HAND_EYE_PARK,
    'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
    'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
    'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def quat_to_matrix(quat_dict):
    """Convert {x, y, z, w} to 3x3 rotation matrix via transforms3d."""
    quat_wxyz = (quat_dict['w'], quat_dict['x'], quat_dict['y'], quat_dict['z'])
    return transforms3d.quaternions.quat2mat(quat_wxyz)


def matrix_to_quat(matrix):
    """Convert 3x3 rotation matrix to (w, x, y, z) via transforms3d."""
    quat_wxyz = transforms3d.quaternions.mat2quat(matrix)
    return quat_wxyz


def extract_opencv_samples(points):
    """
    Extract rotation matrices and translation vectors from points.

    Returns:
        ((hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr))
    where each is a list of 3x3 matrices / 3-vectors respectively.
    """
    hand_world_rot = []
    hand_world_tr = []
    marker_camera_rot = []
    marker_camera_tr = []

    for point in points:
        # robot_transform = base->effector (eye_on_base mode)
        robot_tf = point['robot_transform']
        robot_rot = quat_to_matrix(robot_tf['rotation'])
        robot_tr = np.array([robot_tf['translation']['x'],
                              robot_tf['translation']['y'],
                              robot_tf['translation']['z']], dtype=np.float64)
        hand_world_rot.append(robot_rot)
        hand_world_tr.append(robot_tr)

        # tracking_transform = camera->marker
        tracking_tf = point['tracking_transform']
        tracking_rot = quat_to_matrix(tracking_tf['rotation'])
        tracking_tr = np.array([tracking_tf['translation']['x'],
                                 tracking_tf['translation']['y'],
                                 tracking_tf['translation']['z']], dtype=np.float64)
        marker_camera_rot.append(tracking_rot)
        marker_camera_tr.append(tracking_tr)

    return (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr)


def main():
    parser = argparse.ArgumentParser(
        description='Recalibration offline à partir de samples sauvegardés.')
    parser.add_argument('--input', default=DEFAULT_INPUT,
                        help=f'Fichier YAML input (défaut: {DEFAULT_INPUT})')
    parser.add_argument('--threshold', type=float, default=DEFAULT_THRESHOLD,
                        help=f'Seuil ambiguity_ratio pour inclure un point (défaut: {DEFAULT_THRESHOLD}). '
                             'Utilise >= threshold (inf toujours inclus).')
    parser.add_argument('--method', default=DEFAULT_METHOD,
                        choices=list(AVAILABLE_METHODS.keys()),
                        help=f'Algorithme cv2.calibrateHandEye (défaut: {DEFAULT_METHOD})')
    parser.add_argument('--output', default=None,
                        help='Fichier YAML output pour sauvegarder le résultat. '
                             'Pas de sauvegarde si non fourni.')
    args = parser.parse_args()

    # ── Charger le fichier YAML ───────────────────────────────────────────────
    try:
        with open(args.input, 'r') as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"Erreur: fichier non trouvé: {args.input}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Erreur lecture YAML: {e}", file=sys.stderr)
        sys.exit(1)

    if not data or 'points' not in data:
        print("Erreur: aucun champ 'points' dans le fichier.", file=sys.stderr)
        sys.exit(1)

    all_points = data['points']
    print(f"Total points chargés: {len(all_points)}")

    # ── Filtrer par ambiguity_ratio ───────────────────────────────────────────
    filtered_points = []
    rejected_indices = []

    for point in all_points:
        ratio = point.get('ambiguity_ratio', float('inf'))
        index = point.get('index', '?')

        # Inclure si ratio est inf (ChArUco, pas d'ambiguité) ou >= threshold
        if np.isinf(ratio) or ratio >= args.threshold:
            filtered_points.append(point)
        else:
            rejected_indices.append((index, ratio))

    print(f"\nFiltrage par ambiguity_ratio >= {args.threshold}:")
    print(f"  Gardés:   {len(filtered_points)}")
    print(f"  Rejetés:  {len(rejected_indices)}")
    if rejected_indices:
        print(f"  Indices rejetés:")
        for idx, ratio in rejected_indices:
            print(f"    - point {idx}: ratio={ratio:.2f}")

    # ── Vérifier minimum ──────────────────────────────────────────────────────
    if len(filtered_points) < MIN_SAMPLES:
        print(f"\nErreur: besoin au moins {MIN_SAMPLES} points valides, "
              f"n'en a que {len(filtered_points)}", file=sys.stderr)
        sys.exit(1)

    # ── Extraire et normaliser les données ─────────────────────────────────────
    try:
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = \
            extract_opencv_samples(filtered_points)
    except Exception as e:
        print(f"Erreur extraction samples: {e}", file=sys.stderr)
        sys.exit(1)

    # ── Calibration ───────────────────────────────────────────────────────────
    method_flag = AVAILABLE_METHODS[args.method]
    print(f"\nCalibration (algorithme: {args.method}, {len(filtered_points)} points)...")
    try:
        hand_camera_rot, hand_camera_tr = cv2.calibrateHandEye(
            hand_world_rot, hand_world_tr,
            marker_camera_rot, marker_camera_tr,
            method=method_flag)
    except Exception as e:
        print(f"Erreur cv2.calibrateHandEye: {e}", file=sys.stderr)
        sys.exit(1)

    # ── Convertir le résultat en quaternion ────────────────────────────────────
    quat_wxyz = matrix_to_quat(hand_camera_rot)
    quat_xyzw = (quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0])  # réordonner en (x,y,z,w)

    print(f"\nRésultat:")
    print(f"  Translation (m):  x={hand_camera_tr[0][0]:.6f}  y={hand_camera_tr[1][0]:.6f}  z={hand_camera_tr[2][0]:.6f}")
    print(f"  Quaternion:       x={quat_xyzw[0]:.6f}  y={quat_xyzw[1]:.6f}  z={quat_xyzw[2]:.6f}  w={quat_xyzw[3]:.6f}")

    # ── Sauvegarder si demandé ────────────────────────────────────────────────
    if args.output:
        result_data = {
            'transform': {
                'translation': {
                    'x': float(hand_camera_tr[0][0]),
                    'y': float(hand_camera_tr[1][0]),
                    'z': float(hand_camera_tr[2][0]),
                },
                'rotation': {
                    'x': float(quat_xyzw[0]),
                    'y': float(quat_xyzw[1]),
                    'z': float(quat_xyzw[2]),
                    'w': float(quat_xyzw[3]),
                },
            }
        }
        try:
            with open(args.output, 'w') as f:
                yaml.dump(result_data, f, default_flow_style=False)
            print(f"\n✓ Résultat sauvegardé: {args.output}")
        except Exception as e:
            print(f"Erreur sauvegarde YAML: {e}", file=sys.stderr)
            sys.exit(1)


if __name__ == '__main__':
    main()

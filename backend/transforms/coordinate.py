"""
Coordinate Transformations
=========================

Transform coordinates between camera frame and robot base frame.

Refer to the README for camera mounting specifications.
"""

import numpy as np


def build_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Build a 3x3 rotation matrix from Roll-Pitch-Yaw (Euler) angles.
    
    Args:
        roll: Rotation about X-axis in radians
        pitch: Rotation about Y-axis in radians
        yaw: Rotation about Z-axis in radians
    
    Returns:
        3x3 rotation matrix
    """
    raise NotImplementedError("build_rotation_matrix")


def camera_to_robot(point_camera: np.ndarray) -> np.ndarray:
    """
    Transform a point from camera frame to robot base frame.
    
    Args:
        point_camera: [x, y, z] coordinates in camera frame (mm)
    
    Returns:
        [x, y, z] coordinates in robot base frame (mm)
    """
    raise NotImplementedError("camera_to_robot")


def robot_to_camera(point_robot: np.ndarray) -> np.ndarray:
    """
    Transform a point from robot base frame to camera frame.
    
    Args:
        point_robot: [x, y, z] coordinates in robot base frame (mm)
    
    Returns:
        [x, y, z] coordinates in camera frame (mm)
    """
    raise NotImplementedError("robot_to_camera")


def build_homogeneous_transform(
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """
    Build a 4x4 homogeneous transformation matrix.
    
    Args:
        rotation: 3x3 rotation matrix
        translation: 3x1 or (3,) translation vector
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    raise NotImplementedError("build_homogeneous_transform")

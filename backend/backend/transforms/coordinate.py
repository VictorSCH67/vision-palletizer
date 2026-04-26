"""
Coordinate Transformations
=========================

Transform coordinates between camera frame and robot base frame.

Refer to the README for camera mounting specifications.
"""

import numpy as np
import math


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
    
    cx = np.cos(yaw)
    sx = np.sin(yaw)
    cy = np.cos(pitch)
    sy = np.sin(pitch)
    cz = np.cos(roll)
    sz = np.sin(roll) 
    
    # R = Rz(yaw)*Ry(pitch)*Rx(roll)
    Rotation_Matrix = np.array([
      [cz*cy, cz*sy*sx-cx*sz, cz*cx*sy+sz*sx],
      [cy*sz, sz*sy*sx+cz*cx, cx*sz*sy-cz*sx],    
      [  -sy,          cy*sx,          cy*cx]
    ])
        
    return Rotation_Matrix

    
def build_translation_matrix(x: float, y: float, z: float) -> np.ndarray:
    """
    Build a 3x1 translation matrix from x, y, z coordinates.
    Args:
        x: Position along X-axis in meters
        y: Position along Y-axis in meters
        z: Position along Z-axis in meters
    Returns:
        3x1 translation matrix
    """
    
    Translation_Matrix = np.array([ 
      [x],
      [y],
      [z]
    ])
    
    return Translation_Matrix


def extract_rotation_matrix(transformation: np.ndarray) -> np.ndarray:
    """
    Extract the 3x3 rotation matrix from a 4x4 homogeneous transformation matrix
    Args:
        transformation: 4x4 homogeneous transformation matrix
    Returns:
        3x3 rotation matrix
    """
    
    Rotation_Matrix = np.array([
      [transformation[0][0], transformation[0][1], transformation[0][2]],
      [transformation[1][0], transformation[1][1], transformation[1][2]],
      [transformation[2][0], transformation[2][1], transformation[2][2]]
    ])
    
    return Rotation_Matrix
    
    
def extract_translation_matrix(transformation: np.ndarray) -> np.ndarray:
    """
    Extract the 3x1 rotation matrix from a 4x4 homogeneous transformation matrix
    Args:
        transformation: 4x4 homogeneous transformation matrix
    Returns:
        3x1 translation matrix
    """
    
    Translation_Matrix = np.array([
      [transformation[0][3]],
      [transformation[1][3]],
      [transformation[2][3]]
    ])
    
    return Translation_Matrix
    

def get_euler_angle_ZYX(rotation: np.ndarray) -> np.ndarray:
    """
    From a 3x3 rotation matrix, compute the Euler angle (mobile ZYX = RPY convention)
    Args:
        rotation: 3x3 rotation matrix
    Returns:
        3x1 Euler angle vector (rx, ry, rz)
    """
   
    # Relative error on a double 2^-52
    EPS = 2.2204 * 10**-16
    rcy = math.sqrt(rotation[0][0] * rotation[0][0] + rotation[1][0]*rotation[1][0])
    
    if rcy > EPS:
        Euler_Angles = np.array([
            math.atan2(rotation[1][0], rotation[0][0]),
            math.atan2(-rotation[2][0], rcy),
            math.atan2(rotation[2][1], rotation[2][2])
            ])
    else:
        Euler_Angles = np.array([
            0,
            math.atan2(-rotation[2][1], rcy),
            math.atan2(-rotation[1][2], rotation[1][1])
            ])
    return Euler_Angles


def build_homogeneous_transform(
    rotation: np.ndarray,
    translation: np.ndarray) -> np.ndarray:
    """
    Build a 4x4 homogeneous transformation matrix.
    Args:
        rotation: 3x3 rotation matrix
        translation: 3x1 translation matrix
    Returns:
        4x4 homogeneous transformation matrix
    """
    
    Homogeneous_Matrix = np.array(
    [ [rotation[0][0], rotation[0][1], rotation[0][2], translation[0][0]],
      [rotation[1][0], rotation[1][1], rotation[1][2], translation[1][0]],
      [rotation[2][0], rotation[2][1], rotation[2][2], translation[2][0]],
      [             0,              0,              0,                 1]
    ])
    
    return Homogeneous_Matrix

    
def invert_homogeneous_transform(transformation: np.ndarray) -> np.ndarray:
    """
    Invert a 4x4 homogeneous transformation matrix.
    Args:
        transformation: 4x4 homogeneous transformation matrix
    Returns:
        (transformation)^-1: 4x4 homogeneous transformation matrix
    
    Let H a 4x4 homogeneous transformation matrix, such as:
    H = [R T]
        [0 1]
      
      where: 
      R is a 3x3 rotation matrix, and T is a 3x1 translation matrix.
      0 is a 1x3 zero vector and 1 a scalar.
    
    The invert of H is H^-1, such that:
    H^-1 = [Rt -Rt*T]
           [0      1]
       where:
       Rt is the transpose rotation matrix.
    """
    
    Rt = extract_rotation_matrix(transformation).T
    T = extract_translation_matrix(transformation)
    
    return build_homogeneous_transform(Rt,-Rt @ T)


def camera_to_robot(
    position_camera: np.ndarray,
    rotation_camera: np.ndarray) -> np.ndarray:
    """
    Transform a point from camera frame to robot base frame.
    Args:
        position_camera: [x, y, z] coordinates in camera frame (mm)
        rotation_camera: [roll, pitch, yaw] rotation angle in the camera frame (deg)
    Returns:
        [x, y, z] coordinates in robot base frame (mm)
        [roll, pitch, yaw] rotations in the robot base frame (deg)
    """
    
    # Read inputs and convert them in meters
    Xm = 0.001 * position_camera[0]
    Ym = 0.001 * position_camera[1]
    Zm = 0.001 * position_camera[2]
    Rollrad = np.deg2rad(rotation_camera[0])
    Pitchrad = np.deg2rad(rotation_camera[1])
    Yawrad = np.deg2rad(rotation_camera[2])
    
    # Box pose with respect to the Camera frame
    H_BwrtC = build_homogeneous_transform(build_rotation_matrix(Rollrad, Pitchrad, Yawrad), build_translation_matrix(Xm, Ym, Zm))
    
    # Compute the Box pose with respect to the Robot frame
    H_BwrtR = get_H_CwrtR() @ H_BwrtC
    
    # Get the point's translation with respect to the Robot frame and convert the values in mm
    Translation = 1000 * extract_translation_matrix(H_BwrtR)
    
    # Get the point's rotation in the Euler angle form
    Euler_Anlges_rad = get_euler_angle_ZYX(extract_rotation_matrix(H_BwrtR))
    
    # Convert the Euler angles in deg
    Euler_Angles_deg = [0, 0, 0]
    Euler_Angles_deg[0] = np.rad2deg(Euler_Anlges_rad[0])
    Euler_Angles_deg[1] = np.rad2deg(Euler_Anlges_rad[1])
    Euler_Angles_deg[2] = np.rad2deg(Euler_Anlges_rad[2])
    
    return [Translation[0][0], Translation[1][0], Translation[2][0], Euler_Angles_deg[0] , Euler_Angles_deg[1] , Euler_Angles_deg[2]]


def robot_to_camera(
    point_robot: np.ndarray,
    rotation_robot: np.ndarray) -> np.ndarray:
    """
    Transform a point from robot base frame to camera frame.
    Args:
        point_robot: [x, y, z] coordinates in robot base frame (mm)
        rotation_robot: [roll, pitch, yaw] rotation angle in the robot base frame (deg)
    Returns:
        [x, y, z] coordinates in camera frame (mm)
        [roll, pitch, yaw] rotations in the camera frame (deg)
    """
    
    # Read inputs and convert them in meters
    Xm = 0.001 * point_robot[0]
    Ym = 0.001 * point_robot[1]
    Zm = 0.001 * point_robot[2]
    Rollrad = np.deg2rad(rotation_robot[0])
    Pitchrad = np.deg2rad(rotation_robot[1])
    Yawrad = np.deg2rad(rotation_robot[2])
    
    # Point's pose with respect to the Robot frame 
    H_PwrtR = build_homogeneous_transform(build_rotation_matrix(Rollrad, Pitchrad, Yawrad), build_translation_matrix(Xm, Ym, Zm))
    
    # Compute the point's pose with respect to the Camera frame
    H_PwrtC = invert_homogeneous_transform(get_H_CwrtR()) @ H_PwrtR;
    
    # Get the point's translation with respect to the Camera frame and convert the values in mm
    Translation = 1000 * extract_translation_matrix(H_PwrtC)
    
    # Get the point's rotation in the Euler angle form
    Euler_Angles_rad = get_euler_angle_ZYX(extract_rotation_matrix(H_PwrtC))
    
    # Convert the Euler angles in deg
    Euler_Angles_deg = [0, 0, 0]
    Euler_Angles_deg[0] = np.rad2deg(Euler_Angles_rad[0])
    Euler_Angles_deg[1] = np.rad2deg(Euler_Angles_rad[1])
    Euler_Angles_deg[2] = np.rad2deg(Euler_Angles_rad[2])
    
    return [Translation[0][0], Translation[1][0], Translation[2][0], Euler_Angles_deg[0] , Euler_Angles_deg[1] , Euler_Angles_deg[2]]


def get_H_CwrtR() -> np.ndarray:
    """
    Compute the 4x4 homogeneous matrix that represent the camera transform with respect to the robot reference frame.
    Args:
    Returns:
        4x4 homogeneous matrix
    """
    
    # Camera Mounting pose from the technical specifications
    x = 0.5
    y = 0.3
    z = 0.8
    roll = np.deg2rad(15)
    pitch = np.deg2rad(-10)
    yaw = np.deg2rad(45)
        
    return build_homogeneous_transform(build_rotation_matrix(roll, pitch, yaw), build_translation_matrix(x, y, z))


"""
Motion Controller
================

Implements robot motion commands for pick and place operations.
"""

from typing import Optional
import numpy as np

from .connection import RobotConnection


class MotionController:
    """
    Controls robot motion for palletizing operations.
    
    Coordinates:
    - All positions are in meters
    - All orientations are in radians (axis-angle representation for UR)
    """
    
    # Safety parameters
    APPROACH_HEIGHT_OFFSET = 0.100  # 100mm above pick/place position
    DEFAULT_VELOCITY = 0.5          # m/s
    DEFAULT_ACCELERATION = 0.5      # m/s²
    
    def __init__(self, connection: RobotConnection):
        """
        Initialize motion controller.
        
        Args:
            connection: Active robot connection instance.
        """
        self.connection = connection
        self._gripper_closed = False
    
    def move_to_home(self) -> bool:
        """
        Move robot to home/safe position.
        
        Returns:
            True if move completed successfully.
        """
        raise NotImplementedError("move_to_home")
    
    def move_to_pick(
        self,
        position: list[float],
        orientation: Optional[list[float]] = None,
    ) -> bool:
        """
        Execute pick motion sequence.
        
        Args:
            position: [x, y, z] pick position in robot base frame (meters)
            orientation: [rx, ry, rz] tool orientation (axis-angle, radians)
                        If None, use default downward orientation.
        
        Returns:
            True if pick completed successfully.
        """
        raise NotImplementedError("move_to_pick")
    
    def move_to_place(
        self,
        position: list[float],
        orientation: Optional[list[float]] = None,
    ) -> bool:
        """
        Execute place motion sequence.
        
        Args:
            position: [x, y, z] place position in robot base frame (meters)
            orientation: [rx, ry, rz] tool orientation (axis-angle, radians)
                        If None, use default downward orientation.
        
        Returns:
            True if place completed successfully.
        """
        raise NotImplementedError("move_to_place")
    
    def open_gripper(self) -> bool:
        """
        Open the gripper to release object.
        
        Returns:
            True if gripper opened successfully.
        """
        self._gripper_closed = False
        print("[MOCK] Gripper opened")
        return True
    
    def close_gripper(self) -> bool:
        """
        Close the gripper to grasp object.
        
        Returns:
            True if gripper closed successfully.
        """
        self._gripper_closed = True
        print("[MOCK] Gripper closed")
        return True
    
    def _move_linear(
        self,
        pose: list[float],
        velocity: float = DEFAULT_VELOCITY,
        acceleration: float = DEFAULT_ACCELERATION,
    ) -> bool:
        """
        Execute linear move to target pose.
        
        Args:
            pose: [x, y, z, rx, ry, rz] target pose
            velocity: Move velocity in m/s
            acceleration: Move acceleration in m/s²
        
        Returns:
            True if move completed.
        """
        if self.connection.is_mock_mode():
            print(f"[MOCK] moveL to {pose[:3]}")
            return True
        
        raise NotImplementedError("_move_linear")
    
    def _move_joint(
        self,
        joints: list[float],
        velocity: float = 1.0,
        acceleration: float = 1.0,
    ) -> bool:
        """
        Execute joint move to target configuration.
        
        Args:
            joints: List of 6 joint angles in radians
            velocity: Joint velocity in rad/s
            acceleration: Joint acceleration in rad/s²
        
        Returns:
            True if move completed.
        """
        if self.connection.is_mock_mode():
            print(f"[MOCK] moveJ to {joints}")
            return True
        
        raise NotImplementedError("_move_joint")
    
    def get_default_orientation(self) -> list[float]:
        """
        Get default tool orientation for picking (pointing down).
        
        Returns:
            [rx, ry, rz] in axis-angle representation.
        
        Note: For a tool pointing straight down (Z toward floor),
        the rotation from base frame is typically [0, π, 0] or [π, 0, 0]
        depending on your tool frame setup.
        """
        return [0.0, np.pi, 0.0]

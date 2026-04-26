"""
Motion Controller
================

Implements robot motion commands for pick and place operations.
"""

import time
import math

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
    DEFAULT_VELOCITY = 0.5          # m/s  for a moveL - rad/s² for a moveJ
    DEFAULT_ACCELERATION = 0.5      # m/s² for a moveL - rad/s² for a moveJ
    HIGH_VELOCITY = 0.75            # m/s  for a moveL - rad/s² for a moveJ
    HIGH_ACCELERATION = 0.75        # m/s² for a moveL - rad/s² for a moveJ
    LOW_VELOCITY = 0.25             # m/s  for a moveL - rad/s² for a moveJ
    LOW_ACCELERATION = 0.25         # m/s² for a moveL - rad/s² for a moveJ
    WAIT_TIME = 2                   # secondes
    
    JOINT_TARGET_HOME = [0, -np.pi/2, -np.pi/2, 0, np.pi/2, 0]
    
    
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
        
        # Move in joint space
        if not self._move_joint(self.JOINT_TARGET_HOME, self.HIGH_VELOCITY, self.HIGH_ACCELERATION):
            return False
        
        return True


    def move_to_pick(
        self,
        pose: list[float],
    ) -> bool:
        """
        Execute pick motion sequence.
        
        Args:
            pose: vector that contains position and orientation
                position: [x, y, z] pick position in robot base frame (meters)
                orientation: [rx, ry, rz] tool orientation (axis-angle, radians)
                        If None, use default downward orientation.
        
        Returns:
            True if pick completed successfully.
        """
            
        # Set the position
        position = [0, 0, 0]
        position[0] = pose[0] # x
        position[1] = pose[1] # y
        position[2] = pose[2] # z
        
        # Set the orientation
        orientation = [0, 0, 0]
        if len(pose) < 3:
            # No orientation defined, take the default one.
            orientation = self.get_default_orientation()
        else:
            orientation[0] = pose[3] 
            orientation[1] = pose[4] 
            orientation[2] = pose[5] 
        
        # Waypoint approah pick
        WP_approah_pick = [
            position[0],
            position[1],
            position[2] + self.APPROACH_HEIGHT_OFFSET,
            orientation[0],
            orientation[1],
            orientation[2],
        ]
        
        # Waypoint pick
        WP_pick = [
            position[0],
            position[1],
            position[2],
            orientation[0],
            orientation[1],
            orientation[2],
        ]
        
        # Approach 
        #  Note that, we would have preferred to perform 
        #  a self._move_joint() for the approach.
        #  However, in the current version, self._move_joint()
        #  does not handle the cartesian pose as an input 
        if not self._move_linear(WP_approah_pick, self.HIGH_VELOCITY, self.HIGH_ACCELERATION):
            return False
        
        # Descend
        if not self._move_linear(WP_pick, self.LOW_VELOCITY, self.LOW_ACCELERATION):
            return False
            
        # Close the gripper
        self.close_gripper()
        
        # Delay to insure the gripper is closed
        self.wait(self.WAIT_TIME)
        
        # Retract
        if not self._move_linear(WP_approah_pick, self.LOW_VELOCITY, self.LOW_ACCELERATION):
            return False
            
        return True
        
    
    def move_to_place(
        self,
        pose: list[float]
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
         
        # Get the default orientation if no orientation received
        #if orientation is None:
        #    orientation = self.get_default_orientation()
        
        # Set the position
        position = [0, 0, 0]
        position[0] = pose[0] # x
        position[1] = pose[1] # y
        position[2] = pose[2] # z
        
        # Set the orientation as the default orientation for placing
        orientation = self.get_default_orientation()
        
        # Waypoint approah place
        WP_approah_place = [
            position[0],
            position[1],
            position[2] + self.APPROACH_HEIGHT_OFFSET,
            orientation[0],
            orientation[1],
            orientation[2],
        ]
        
        # Waypoint place
        WP_place = [
            position[0],
            position[1],
            position[2],
            orientation[0],
            orientation[1],
            orientation[2],
        ]
        
        # Approach 
        #  Note that, we would have preferred to perform 
        #  a self._move_joint() for the approach.
        #  However, in the current version, self._move_joint()
        #  does not handle the cartesian pose as an input 
        if not self._move_linear(WP_approah_place, self.HIGH_VELOCITY, self.HIGH_ACCELERATION):
            return False
        
        # Descend
        if not self._move_linear(WP_place, self.LOW_VELOCITY, self.LOW_ACCELERATION):
            return False
            
        # Open the gripper
        self.open_gripper()
        
        # Delay to insure the gripper is opened
        self.wait(self.WAIT_TIME)
        
        # Retract
        if not self._move_linear(WP_approah_place, self.LOW_VELOCITY, self.LOW_ACCELERATION):
            return False
            
        return True
        
    
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
    

    def wait(self, duration: float) -> None:
        """
        Pause execution for a specified duration.

        Args:
            duration: Time to wait in seconds.
        """
        
        print(f"[MOCK] Waiting for {duration:.2f} seconds...")
        time.sleep(duration)
    
    
    def _move_linear(
        self,
        pose: list[float],
        velocity: float,
        acceleration: float,
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
        
        # Validate the position against the robot workspace
        if not is_target_position_in_workspace(pose[0], pose[1], pose[2]):
            return False
        
        if self.connection.is_mock_mode():
            print(f"[MOCK] moveL to {print_format(pose)}")
            return True
        
        if not self.connection.ensure_connected() or not self.connection.control:
            return False

        if not self.connection.control.moveL(pose, velocity, acceleration):
            return False
         
        return True

    
    def _move_joint(
        self,
        joints: list[float],
        velocity: float,
        acceleration: float,
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
            if joints == self.JOINT_TARGET_HOME:
                print(f"[MOCK] moveJ HOME {print_format(joints)}")
            else:
                print(f"[MOCK] moveJ to {print_format(joints)}")
            return True
       
        if not self.connection.ensure_connected() or not self.connection.control:
            return False
            
        if not self.connection.control.moveJ(joints, velocity, acceleration):
            return False
            
        return True
        
    
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
        
        
    def is_moving(self) -> bool:
        """
        Returns True if the robot is currently moving.
        """
        
        if self.connection.is_mock_mode():
            time.sleep(1)
        
        return True

    
def print_format(input_list: list[float], decimals: int = 6) -> str:
    """
    Print format for [MOCK] motions
    """
    return "[" + ", ".join(f"{float(p):10.{decimals}f}" for p in input_list) + "]"


def is_target_position_in_workspace(x: float, y: float, z: float) -> bool:
        """
        Check if the position is within the robot envelope.
        Considering a UR5e: 
         - its maximum range is 0.850 m: the robot cannot go further
         - close to the robots base, the cartesian motion my by unstable, 
           we avoid to approach closer to 0.100 m from the robot base
        
        Args:
            position: [x, y, z] target position
        
        Returns:
            True if the position is within the workspace.
        """
        # UR5 maximum reach
        maximum_reach = 0.85
        # UR5 hollow to avoid
        cylinder_radius = 0.2
        
        # Avoid the cylinder area on top of the base
        if math.sqrt(x*x + y*y) < cylinder_radius:
            # Shoulder singularity.
            print("[MOCK]: To close from the robots base.")
            return False
        
        if math.sqrt(x*x + y*y + z*z) > maximum_reach:
            # Elbow singularity.
            print("[MOCK]: The robot is fully stretched and cannot go further.")
            print(x)
            print(y)
            print(z)
            return False
        
        # The position is reachable.
        return True
            
        


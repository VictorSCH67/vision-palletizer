"""
Palletizer API Routes
====================

FastAPI routes for palletizer control.
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional,  List, Tuple 
import numpy as np

import json
from pathlib import Path


from transforms.coordinate import robot_to_camera
from transforms.coordinate import camera_to_robot

from palletizer.grid import calculate_place_positions
from palletizer.grid import get_grid_size

from robot.motion import MotionController
from robot.connection import RobotConnection

from palletizer.state_machine import PalletizerStateMachine

# Create a connection (real or mock)
conn = RobotConnection() 

# Create the controller
controller = MotionController(conn)

# Create the palletizer state machine
sm = PalletizerStateMachine(motion_controller=controller)

# ============================================================================
# Global
# ============================================================================

# API Router
router = APIRouter()

# IP adress of the simulator
ROBOT_IP = "172.18.0.2"
DASHBOARD_PORT = 29999


# Palletizer state
PALLETIZER_STATE_NOT_INIT = "Palletizer_Not_Initialized"
PALLETIZER_STATE_GRID_CONFIGURED = "Palletizer_Grid_Configured"
palletizer_state = {
    "state": PALLETIZER_STATE_NOT_INIT,
    "current_box": -1,
    "total_boxes": -1,
    "error": None
    }
    

# ============================================================================
# Request/Response Models
# ============================================================================

class PalletConfig(BaseModel):
    """Configuration for palletizing operation."""
    
    rows: int = Field(..., ge=1, le=10, description="Number of rows in the grid")
    cols: int = Field(..., ge=1, le=10, description="Number of columns in the grid")
    box_width_mm: float = Field(..., gt=0, description="Box width in mm (X direction)")
    box_depth_mm: float = Field(..., gt=0, description="Box depth in mm (Y direction)")
    box_height_mm: float = Field(..., gt=0, description="Box height in mm (Z direction)")
    pallet_origin_x_mm: float = Field(..., description="Pallet origin X in mm")
    pallet_origin_y_mm: float = Field(..., description="Pallet origin Y in mm")
    pallet_origin_z_mm: float = Field(..., description="Pallet origin Z in mm")
    
    class Config:
        json_schema_extra = {
            "example": {
                "rows": 2,
                "cols": 2,
                "box_width_mm": 100.0,
                "box_depth_mm": 100.0,
                "box_height_mm": 50.0,
                "pallet_origin_x_mm": 400.0,
                "pallet_origin_y_mm": -200.0,
                "pallet_origin_z_mm": 100.0,
            }
        }


class VisionDetection(BaseModel):
    """Simulated vision detection of a box."""
    
    x_mm: float = Field(..., description="Box X position in camera frame (mm)")
    y_mm: float = Field(..., description="Box Y position in camera frame (mm)")
    z_mm: float = Field(..., description="Box Z position in camera frame (mm)")
    yaw_deg: Optional[float] = Field(0.0, description="Box rotation about Z (degrees)")
    
    class Config:
        json_schema_extra = {
            "example": {
                "x_mm": 50.0,
                "y_mm": -30.0,
                "z_mm": 0.0,
                "yaw_deg": 15.0,
            }
        }


class StatusResponse(BaseModel):
    """Palletizer status response."""
    
    state: str = Field(..., description="Current state machine state")
    current_box: int = Field(..., description="Current box index (0-based)")
    total_boxes: int = Field(..., description="Total boxes to palletize")
    error: Optional[str] = Field(None, description="Error message if in FAULT state")


class ConfigResponse(BaseModel):
    """Configuration response."""
    
    success: bool
    message: str
    grid_size: Optional[str] = None


class CommandResponse(BaseModel):
    """Generic command response."""
    
    success: bool
    message: str


# ============================================================================
# API Endpoints
# ============================================================================

@router.post("/configure", response_model=ConfigResponse)
async def configure_palletizer(config: PalletConfig):
    """
    Configure the palletizing operation.
    
    Sets up the grid dimensions, box size, and pallet origin.
    Can only be called when the palletizer is in IDLE state.
    """  
    global TCP_Places_Poses
    global Box_Height
    
    # Set the box size
    BoxSize = (config.box_width_mm, config.box_depth_mm, config.box_height_mm)
    
    # Set the pallet origin
    PalletOrigin = (config.pallet_origin_x_mm, config.pallet_origin_y_mm, config.pallet_origin_z_mm) 
    
    # Configure the state machine
    success = sm.configure(
        rows = config.rows,
        cols = config.cols,
        box_size_mm = BoxSize,
        pallet_origin_mm = PalletOrigin
    )
    
    # Verify that the state machine is configured
    if not success:
        return {
        "success": False,
        "message": "Cannot configure while palletizer is running.",
        "grid_size": None}

    # Get all the TCP place poses
    TCP_Places_Poses = calculate_place_positions(config.rows, config.cols, BoxSize, PalletOrigin)
    
    # Verify that TCP Target_Poses is not empty 
    if not TCP_Places_Poses or len(TCP_Places_Poses) == 0:
        return {
        "success": False,
        "message": "Positions not calculated.",
        "grid_size": "X_grid: 0mm, Y_grid: 0mm, Layers: 0"
    }
    
    # Set the TCP places poses to the state machine
    sm.context.place_positions = TCP_Places_Poses    
        
    # Compute the grid size from the TCP target positions
    Xg_mm, Yg_mm, NB_Layers = get_grid_size(TCP_Places_Poses)
    
    # Update the palletizer state
    palletizer_state["state"] = PALLETIZER_STATE_GRID_CONFIGURED
    palletizer_state["current_box"] = 0
    palletizer_state["total_boxes"] = config.rows * config.cols * NB_Layers

    # Set the TCP pick poses to the state machine
    Box_Height = 0.001 * config.box_height_mm
    await read_camera_detection_json()
    
    return {
        "success": True,
        "message": "Positions calculated successfully",
        "grid_size": f"X_grid: {Xg_mm:.0f}mm, Y_grid: {Yg_mm:.0f}mm, Layers: {NB_Layers}"
    }
    

@router.post("/start", response_model=CommandResponse)
async def start_palletizer():
    """
    Start the palletizing sequence.
    
    Begins the pick-and-place cycle. The palletizer must be configured first.
    """
    # Check for the palletizer state
    if not palletizer_state["state"] == PALLETIZER_STATE_GRID_CONFIGURED:
        return {"success": False, "message": "Palletizer not configured, cannot start."}
        
    # Start the palletizer sequence
    sm.begin()
    return {"success": True, "message": "Palletizer started."}   
    

@router.post("/stop", response_model=CommandResponse)
async def stop_palletizer():
    """
    Stop the palletizing sequence.
    
    Gracefully stops the operation and returns to IDLE state.
    """
    
    if sm.stop():
        return {"success": True, "message": "Palletizer stopped."}
        
    return {"success": False, "message": "Failed to stop palletizer."}


@router.post("/reset", response_model=CommandResponse)
async def reset_palletizer():
    """
    Reset from FAULT state.
    
    Clears the fault and returns to IDLE state.
    """
    
    if sm.reset():
        # Reset the pick positions list
        sm.context.pick_positions = []
        return {"success": True, "message": "Palletizer reset to IDLE."}
        
    return {"success": False, "message": "Failed to reset palletizer."}


@router.get("/status", response_model=StatusResponse)
async def get_status():
    """
    Get current palletizer status.
    
    Returns the current state, progress, and any error messages.
    """
    
    progress = sm.progress
    return StatusResponse(
        state=progress["state"],
        current_box=progress["current_box"],
        total_boxes=progress["total_boxes"],
        error=progress["error"])
    

@router.post("/vision/detect", response_model=CommandResponse)
async def simulate_vision_detection(detection: VisionDetection):
    """
    Simulate a vision detection event.
    
    In a real system, this would come from the vision system.
    For this exercise, use this endpoint to simulate box detections.
    
    The coordinates are in the camera frame and must be transformed
    to the robot frame before use.
    """
    global Box_Height
    
    try:
        Box_Height
    except NameError:
        return {
            "success": False,
            "message": "Failed because the palletizer is not configured."
        }
    
    # Compute the input coordinates in the robot frame
    robot_frame = camera_to_robot([detection.x_mm, detection.y_mm, detection.z_mm], [0, 0, detection.yaw_deg])
    
    # Check for null list
    if sm.context.pick_positions == None:
        sm.context.pick_positions = []
        
    # Set the pick pose to the state machine 
    sm.context.pick_positions.append([
        0.001 * robot_frame[0], 
        0.001 * robot_frame[1], 
        Box_Height, 
        np.deg2rad(robot_frame[3]), 
        np.deg2rad(robot_frame[4]), 
        np.deg2rad(robot_frame[5])
    ])
    
    return {
        "success": True,
        "message": "Box detected and added to queue."
    }
    

# ============================================================================
# Helper/Debug Endpoints (Optional)
# ============================================================================

@router.get("/debug/positions")
async def get_calculated_positions():
    """
    Debug endpoint: Get all calculated place positions.
    
    Useful for verifying grid calculations without running the full sequence.
    """
    global TCP_Places_Poses
 
    # Check for the palletizer state
    if not palletizer_state["state"] == PALLETIZER_STATE_GRID_CONFIGURED:
        return {"success": False, "message": "Palletizer not configured, cannot get the positions."}
    
    grid_lines = []
    for idx, pos in enumerate(TCP_Places_Poses):
        grid_lines.append(
            f"    Position {idx+1}:"
            f"    x_mm: {pos[0]}"
            f"    y_mm: {pos[1]}"
            f"    z_mm: {pos[2]}"
        )
    return "\n".join(grid_lines)


@router.post("/debug/transform")
async def test_transform(detection: VisionDetection):
    """
    Debug endpoint: Test coordinate transformation.
    
    Transforms the input coordinates and returns both camera and robot frame values.
    Useful for verifying transformation math.
    """
    # Compute the input coordinates in the robot frame
    robot_frame = camera_to_robot([detection.x_mm, detection.y_mm, detection.z_mm], [0, 0, detection.yaw_deg])
    
    # Compute the input coordinates in the camera frame
    camera_frame = robot_to_camera([detection.x_mm, detection.y_mm, detection.z_mm], [0, 0, detection.yaw_deg])
    
    # Return the output
    return {
        "Vision detection input": 
        {
            "x_mm": detection.x_mm,
            "y_mm": detection.y_mm,
            "z_mm": detection.z_mm,
            "yaw_deg": detection.yaw_deg,
        },
        "Input pose with respect to the Robot frame": 
        {
            "x_mm": robot_frame[0],
            "y_mm": robot_frame[1],
            "z_mm": robot_frame[2],
            "rx_deg": robot_frame[3],
            "ry_deg": robot_frame[4],
            "rz_deg": robot_frame[5],
        },
        "Input pose with respect to the Camera frame": 
        {
            "x_mm": camera_frame[0],
            "y_mm": camera_frame[1],
            "z_mm": camera_frame[2],
            "rx_deg": camera_frame[3],
            "ry_deg": camera_frame[4],
            "rz_deg": camera_frame[5],
        }
    }
   
   
# ============================================================================
# Other methods
# ============================================================================

async def read_camera_detection_json():
    
    """
    From the file camera_detections.json, read all TCP pick positions and 
    run simulate_vision_detection() to simulate a camera detection event.
    """
    
    # Path to camera_detection.json
    path = Path(__file__).parent.parent / "data" / "camera_detections.json"

    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")
        
    # Open and read the file
    with open(path, "r") as f:
        data = json.load(f)  
        
    detections = data.get("detections", [])
    
    for det in detections:
        detection_obj = VisionDetection(
            x_mm=det["x_mm"],
            y_mm=det["y_mm"],
            z_mm=det["z_mm"],
            yaw_deg=det.get("yaw_deg", 0.0))
        await simulate_vision_detection(detection_obj)
    
  

"""
Palletizing Grid Calculations
============================

Calculate place positions for boxes in an N×M grid pattern.
"""

from typing import List, Tuple


def calculate_place_positions(
    rows: int,
    cols: int,
    box_size_mm: Tuple[float, float, float],
    pallet_origin_mm: Tuple[float, float, float],
    spacing_mm: float = 10.0,
    layers: int = 1
) -> List[Tuple[float, float, float]]:
    """
    Calculate TCP positions for placing boxes in a grid pattern.
    
    Args:
        rows: Number of rows (N)
        cols: Number of columns (M)
        box_size_mm: (width, depth, height) of each box in mm
        pallet_origin_mm: (x, y, z) position of the first box placement with respect to the robot frame
        spacing_mm: Gap between adjacent boxes (default 10mm)
        layers: Number of layers (K) (default 1)
    
    Returns:
        positions: List of (x, y, z) TCP target positions, ordered for row-by-row filling in meters.
    """
    
    # Box size (in meters)
    BoxSizeX = box_size_mm[0] * 0.001
    BoxSizeY = box_size_mm[1] * 0.001
    BoxSizeZ = box_size_mm[2] * 0.001
    
    # Pallet origin (in meters)
    OriginX = pallet_origin_mm[0] * 0.001
    OriginY = pallet_origin_mm[1] * 0.001
    OriginZ = pallet_origin_mm[2] * 0.001
    
    # Init the output list
    pose: List[Tuple[float, float, float, 0, 0, 0]] = []
    
    for k in range(layers):
        for n in range(rows):
            for m in range(cols):
                x = OriginX + n * (BoxSizeX + spacing_mm * 0.001) 
                y = OriginY + m * (BoxSizeY + spacing_mm * 0.001) 
                # No spacing on Z since on layer is directly on top of the previous one
                # Add the box height
                z = OriginZ + BoxSizeZ * (k + 1)  
                pose.append((x, y, z, 0, 0, 0))
            
    return pose


def get_grid_size(
positions: List[Tuple[float, float, float]]
) -> Tuple[float, float, int]:
    """
    Compute the grid dimensions (xg_mm, yg_mm, nb_layers) from a list of TCP positions.

    Args:
        positions: List of (x, y, z) positions in meters, row-by-row, layer-by-layer.

    Returns:
        Tuple of (xg_mm, yg_mm, layers):
            xg_mm: grid size along X (mm)
            yg_mm: grid size along Y (mmm)
            nb_layers: number of layers
    """
    
    # Check for a valid input
    if not positions:
        return 0.0, 0.0, 0.0

    # Extract unique coordinates in each axis
    xs = sorted(set(pos[0] for pos in positions))
    ys = sorted(set(pos[1] for pos in positions))
    zs = sorted(set(pos[2] for pos in positions))

    # Grid size along X
    xg_mm = 1000 * (max(xs) - min(xs)) if len(xs) > 1 else 0.0
    
    # Grid size along Y
    yg_mm = 1000 * (max(ys) - min(ys)) if len(ys) > 1 else 0.0
    
    # Number of layers
    nb_layers = len(zs)

    return xg_mm, yg_mm, nb_layers



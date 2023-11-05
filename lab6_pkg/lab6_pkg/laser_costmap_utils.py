from typing import Tuple
import numpy as np
import math
from dataclasses import dataclass

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

# def meters_to_pixels(x: float, resolution_p_m: float) -> int:
#     """Converts a measurement in meters along the x or y axis of a grid to the
#     equivalent number of pixels in a 2D grid with a resolution_p_m (given in
#     pixels or cells per meter).

#     Args:
#         x (float): Measurement along the x or y axis given in meters.
#         resolution_m_p (float): The resolution of the grid/map given in pixels
#         per meter.

#     Returns:
#         int: _description_
#     """
    
#     # Resolution given in meters / cell. Flip for cells/meter.
#     return np.floor(x*(1/resolution_m_p))


@dataclass
class ContinuousPosition:
    """Small helper dataclass to maintain a continous 2D position."""
    x: float = 0.0
    y: float = 0.0

# 1. Compute the (continous) X,Y point in the laser frame for each range, theta
#    in the provided scan.
def position_from_range(range_m: float, angle_rad: float) -> ContinuousPosition:
    """Computes a (continuous) 2D position in the laser frame from a range and
    the angle it was measured at. 

    Args:
        range_m (float): The range in meters.
        angle_rad (float): The angle the range was measured at in radians.

    Returns:
        ContinuousPosition: The 2D coordinates of where the point lands in the
        laser frame's x-y plane.
    """
    # Create new empty position.
    position = ContinuousPosition
    # Compute x as the range_m*cosine(angle_rad).
    position.x = range_m*np.cos(angle_rad)
    # Compute y as the range_m*sin(angle_rad).
    position.y = range_m*np.sin(angle_rad)
    return position

@dataclass
class GridPosition:
    x: int = 0
    y: int = 0

def project_continous_point_to_grid(grid_resolution_m_c: float, 
                                    continous_point: ContinuousPosition) -> GridPosition:
    """Takes a continous 2D position in a frame and converts it to coordinates
    in a 2D grid, where there is no transformation (translation or rotation)
    from the source frame to the grid's frame.

    Args:
        grid_resolution_m_c (float): The number of cells per meter in the grid
        you're projecting the continous point into.
        continous_point (ContinuousPosition): Continous position (in meters) of a
        point in an arbitrary frame.

    Returns:
        GridPosition: The position of the continuous point projected into the
        grid. The coordinates with respect to a grid whose 
    """
    pass

# Okay, so, I have the sequence of steps that'll be needed to build up an
# occupancy grid from laser scans. However, now, I want to think not just about
# the sequence of events (roughly), but now about the architecture. I.e., what
# functions/modules do I need to carry this out? Let's start from the top down
# like we would in a formal design process. Start with high level requirements
# and work backwards, breaking down each consecutive level of abstraction into
# intuitive, useful smaller units.


# Given a laser scan message and an existing Occupancy Grid, update the
# occupancy grid with points from the LiDAR.
def laser_update_occupancy_grid(scan_message: LaserScan,
                                current_occupancy_grid: OccupancyGrid) -> OccupancyGrid:
    

    # Within this function, I think we can implement that sequence I have in my
    # notes.

    # 1. Compute the (continous) X,Y point in the laser frame for each range, theta
    #    in the provided scan. TODO: Could wrap this up into a function for
    #    testability. TODO: Consider refactoring with numpy? I.e., doing all
    #    those on the array at once.
    positions = []
    for i, range in enumerate(scan_message.ranges):
        # TODO: Need to call in LiDAR utils function that gives us the
        continous_position = position_from_range(range_m=range, angle_rad=i*scan_message.angle_increment)
        positions.append(continous_position)

    # 2. Given the resolution of the occupancy grid that you're mapping into, assume
    #    the origin of the occupancy grid coincides with the origin of the laser
    #    frame. Using the resolution, compute what the cporresponding (discrete) x,
    #    y indices are for that point as projected into the grid. 
    
    # 3. Now, you have the coordinates of the cell that point corresponds to in
    #    the occupancy grid, assuming that the origin of the laser's frame and
    #    the origin of the occupancy grid are the same. BUT, in fact, there's a
    #    transformation between them. Take those points and apply a translation
    #    and rotation to get them in terms of the actual occupancy grid origin
    #    (I.e., rows and columns of the 2D array).
    
    pass

# This is going to be a QUICK implementation of the more robust implementation
# above. If this works, I can re-implement this in a cleaner way (I.e., breaking
# things up like above).
def laser_update_occupancy_grid_temp(scan_message: LaserScan,
                                     current_occupancy_grid: OccupancyGrid, logger) -> OccupancyGrid:
    
    # Iterate through the scan messages, compute the continuous position of each
    # of them.
    ranges_m = np.array(scan_message.ranges)
    angles_rad = np.array([i*scan_message.angle_increment for i in range(len(ranges_m))])
    points_x_m = np.cos(angles_rad)*ranges_m
    points_y_m = np.sin(angles_rad)*ranges_m
    
    # Compute where these continuous points are in the grid.
    # Get resolution of grid in meters per cell. Flip to get cells per meter.
    grid_resolution_c_m = 1.0/current_occupancy_grid.info.resolution
    x_cell_coords = grid_resolution_c_m*points_x_m
    y_cell_coords = grid_resolution_c_m*points_y_m
    # Now, have to transform those coordinates to the grid's frame. This
    # involves adding the translation from grid origin to laser origin to each
    # coordinate. Before that, though, have to invert both x and y coord, as the
    # grid frame exact opposite basis vector directions for x and y axis.
    height = current_occupancy_grid.info.height
    width = current_occupancy_grid.info.width
    x_cell_coords: np.ndarray = -x_cell_coords + np.floor(height/2)
    y_cell_coords: np.ndarray = -y_cell_coords + np.floor(width/2)
    # Convert these to integer coordinate indices.
    x_cell_coords = x_cell_coords.astype(dtype=np.int32)
    y_cell_coords = y_cell_coords.astype(dtype=np.int32)
    # Clamp the coordinates to the max dimensions of the grid.
    x_cell_coords = np.clip(x_cell_coords, a_min=0, a_max=height-1)
    y_cell_coords = np.clip(y_cell_coords, a_min=0, a_max=width-1)
    # Finally, update the cells at the coordinates in the grid that these LiDAR
    # points fall into.
    # BUT FIRST, have to initialize the data array.
    numpy_occupancy_grid = np.zeros(shape=(height, width), dtype=np.int8)
    # NOTE: Row major indexing. Need to multiply by "width" by number in x, and
    # then add number in y.
    for i in range(len(x_cell_coords)):
        # current_occupancy_grid.data[x_cell_coords[i]*width+y_cell_coords[i]] =
        # 1
        # NOTE: I had the above wrong. I was not following row major order
        # above.
        numpy_occupancy_grid[x_cell_coords[i], y_cell_coords[i]] = 1
    # Reshape 2D numpy array into row-major order to store in occupancygrid
    # array.
    # current_occupancy_grid.data = list(numpy_occupancy_grid.flatten())
    current_occupancy_grid.data = [100]
    

    # Return updated current occupancy grid.
    return current_occupancy_grid
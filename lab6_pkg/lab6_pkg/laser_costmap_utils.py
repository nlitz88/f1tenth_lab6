from typing import Tuple
import numpy as np
import math
from dataclasses import dataclass

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

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

def position_in_grid_frame(position: ContinuousPosition, grid_origin_position: Point) -> ContinuousPosition:
    """Returns the position provided with respect to the parent frame of the
    occupancy grid (laser) and returns it expressed in terms of / from the
    perspective of the occupancy grid's origin.

    Args:
        position (ContinuousPosition): _description_
        grid_origin_position (Point): _description_

    Returns:
        ContinuousPosition: _description_
    """
    pass

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

    # COULD USE THE NEW METHOD ABOVE WHERE WE ACKNOWLEDGE THAT THERE IS ONLY A
    # TRANSLATION, BUT NO ROTATIONS.
    
    pass

# This is going to be a QUICK implementation of the more robust implementation
# above. If this works, I can re-implement this in a cleaner way (I.e., breaking
# things up like above).
def laser_update_occupancy_grid_temp(scan_message: LaserScan,
                                     current_occupancy_grid: OccupancyGrid, logger) -> OccupancyGrid:
    

    # 1.) Get coordinates of where (in range) Lidar scan ranges are projected
    # into the 2D occupancy grid's coordinate system.
    
    # Iterate through the scan messages, compute the continuous position of each
    # of them.
    ranges_m = np.array(scan_message.ranges)
    angles_rad = np.array([i*scan_message.angle_increment + scan_message.angle_min for i in range(len(ranges_m))])
    points_x_m = np.cos(angles_rad)*ranges_m
    points_y_m = np.sin(angles_rad)*ranges_m

    # Transform these points so that they are expressed from the perspective of
    # the occupancy grid's frame/origin. In this case, this just means applying
    # a translation equal to the position of the grid's origin with respect to
    # the parent (laser) frame.
    points_x_m = points_x_m - current_occupancy_grid.info.origin.position.x
    points_y_m = points_y_m - current_occupancy_grid.info.origin.position.y

    # Compute where these continuous points are in the grid.
    # Get resolution of grid in meters per cell. Flip to get cells per meter.
    grid_resolution_c_m = 1.0/current_occupancy_grid.info.resolution
    x_cell_coords = np.array(grid_resolution_c_m*points_x_m, dtype=np.int32)
    y_cell_coords = np.array(grid_resolution_c_m*points_y_m, dtype=np.int32)
    
    # Filter out the coordinates that fall outside of the grid.
    # Actually, needs to be done differently. I.e., if it's out of the range,
    # then I want to completely drop these coordinates--I don't even want them
    # being drawn on the grid. I.e., I don't want all the filtered out points to
    # just be hanging around at 0,0 right in front of the car!
    # Use numpy boolean array indexing to select only the x and y coordinates
    # that fall within the width and height of the grid, respectively.

    # UPDATE: Found a bug. Basically, if I can't filter these two out
    # separately. I.e., if the ith x cell coord is out of bounds, then the
    # corresponding ith y coord also needs to go--as the pair of them (that
    # point) is being discarded.
    cell_coords = np.stack(arrays=[x_cell_coords, y_cell_coords], axis=1)
    filtered_cell_coords = cell_coords[(cell_coords[:, 0] <= current_occupancy_grid.info.width-1)*(cell_coords[:, 0] >= 0)*\
                                       (cell_coords[:, 1] <= current_occupancy_grid.info.height-1)*(cell_coords[:, 1] >= 0)]

    # 2. Once we have the indices of the points of where the LiDAR scan ranges
    #    fall in the occupancy grid, we can now draw them on the occupancy grid.
    
    # Create a 2D numpy array to serve as an easier to work with, temporary
    # occupancy grid that we'll convert to our actual grid later.
    numpy_occupancy_grid = np.zeros(shape=(current_occupancy_grid.info.height, current_occupancy_grid.info.width), dtype=np.int8)

    # TODO Could probably replace this for loop with a numpy operation to index
    # the 2D array using the x_cell_coords and y_cell_coords array.
    for cell_coords in filtered_cell_coords:
        # TODO: Add "splatting" here as a way to "inflate" observed obstacles.
        # NOTE: I'm wondering: numpy uses indexing [row, column]--but we know
        # that when we flatten this, it's expecting that x values are referring
        # to the column instead. I didn't think we'd have to worry about that
        # here. Well, rather, remember how the theme has been that the x basis
        # vector has effectively been acting like our width? (even though it's
        # called height?). Maybe same idea applies here.
        numpy_occupancy_grid[cell_coords[1], cell_coords[0]] = 100

    # Once all the LiDAR landing locations have been marked, can flatten this
    # array and assign it to the occupancy grid.
    current_occupancy_grid.data = numpy_occupancy_grid.flatten().tolist()

    # Return updated current occupancy grid.
    return current_occupancy_grid
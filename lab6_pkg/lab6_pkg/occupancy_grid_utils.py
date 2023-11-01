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
class ContinousPosition:
    """Small helper dataclass to maintain a continous 2D position."""
    x: float
    y: float

# 1. Compute the (continous) X,Y point in the laser frame for each range, theta
#    in the provided scan.
def position_from_range(range_m: float, angle_rad: float) -> ContinousPosition:
    """Computes a (continuous) 2D position in the laser frame from a range and
    the angle it was measured at. 

    Args:
        range_m (float): The range in meters.
        angle_rad (float): The angle the range was measured at in radians.

    Returns:
        ContinousPosition: The 2D coordinates of where the point lands in the
        laser frame's x-y plane.
    """
    # Create new position, initialized at 0, 0.
    position = ContinousPosition(0.0, 0.0)
    # Compute x as the range_m*cosine(angle_rad).
    position.x = range_m*np.cos(angle_rad)
    # Compute y as the range_m*sin(angle_rad).
    position.y = range_m*np.sin(angle_rad)
    return position

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
    #    in the provided scan.

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
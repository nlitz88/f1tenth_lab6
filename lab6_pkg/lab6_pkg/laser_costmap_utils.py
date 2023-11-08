from typing import List, Tuple
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

@dataclass
class GridPosition:
    x: int = 0
    y: int = 0

def project_grid_point_to_continuous(occupancy_grid: OccupancyGrid,
                                     grid_position: Tuple[int, int]) -> Tuple[float, float]:
    """Takes a discrete x,y position in an occupancy grid and projects it back
    into the continuous frame that it's origin is located in/w.r.t.

    Args:
        occupancy_grid (OccupancyGrid): The occupancy grid the point is being
        projected from.
        grid_position (Tuple[int, int]): The position being projected from the
        the occupancy grid to the continuous parent frame.

    Returns:
        Tuple[float, float]: The (continuous) position of the grid position
        projected into x,y coordinates of the parent frame.
    """
    # 1. Take grid position in # cells and multiply by the resolution of the
    #    grid in meters per cell to get the coordinates in meters first.
    grid_x, grid_y = grid_position
    continuous_x = grid_x*occupancy_grid.info.resolution
    continuous_y = grid_y*occupancy_grid.info.resolution
    # 2. Offset the points now in the continuous frame by the position of the
    #    origin of the occupancy grid in that frame. I.e., if a point was at 0,0
    #    in the occupancy grid, it's really at the position of the origin of the
    #    occupancy grid. Add the origin of the occupancy grid to the point.
    offset_x = continuous_x + occupancy_grid.info.origin.position.x
    offset_y = continuous_y + occupancy_grid.info.origin.position.y
    return (offset_x, offset_y)

def project_continuous_point_to_grid(occupancy_grid: OccupancyGrid,
                                     continuous_point: Tuple[float, float]) -> Tuple[int, int]:
    """Takes a continuous 2D position in the same frame as an occupancy grid and
    projects it onto a cell in the occupancy grid.

    Args:
        occupancy_grid (OccupancyGrid): Occupancy grid that the position should
        be projected into.
        continuous_point (Tuple[float, float]): The point as expressed in terms
        of the parent frame of the provided occupancy grid.

    Raises:
        Exception: If the point projects to a cell location that goes out of
        bounds of the occupancy grid, throws an exception.
        
    Returns:
        Tuple[int, int]: The x,y coordinates as grid-cell coordinates, or cell
        offsets from the grid's origin.
    """
    # First, offset the point by the occupancy grid's origin position in its
    # parent frame.
    offset_point_x = continuous_point[0] - occupancy_grid.info.origin.position.x
    offset_point_y = continuous_point[1] - occupancy_grid.info.origin.position.y
    resolution_c_m = 1.0/occupancy_grid.info.resolution
    point_cell_x = int(offset_point_x*resolution_c_m)
    point_cell_y = int(offset_point_y*resolution_c_m)
    # Check if the projection is within the bounds of the occupancy grid.
    if (point_cell_x < 0 or point_cell_x > occupancy_grid.info.width - 1) or \
        (point_cell_y < 0 or point_cell_y > occupancy_grid.info.height - 1):
        raise Exception("Provided point projects to location outside of occupancy grid!")
    return (point_cell_x, point_cell_y)
    
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

def filter_grid_coordinates(cell_coordinates: np.ndarray, grid_width: int, grid_height: int) -> np.ndarray:
    """Filters out (x,y) coordinate pairs from the provided cell_coordinates 2D
    numpy array if they're out of bounds of the occupancy grid, according to the
    specified height and width of the grid. 

    Args:
        cell_coordinates (np.ndarray): 2D numpy array of grid cell coordinates.
        grid_width (int): The number of grid cells along x-axis.
        grid_height (int): The number of grid cells along the y-axis.

    Returns:
        np.ndarray: A subset of the provided cell coordinates that fall within
        the grid.
    """
    return cell_coordinates[(cell_coordinates[:, 0] <= grid_width-1)*(cell_coordinates[:, 0] >= 0)*\
                                       (cell_coordinates[:, 1] <= grid_height-1)*(cell_coordinates[:, 1] >= 0)]


def twod_numpy_from_occupancy_grid(occupancy_grid: OccupancyGrid) -> np.ndarray:
    """Returns a 2D numpy array constructed from the row-major data field of the
    provided occupancy grid. Note that the 2D array returned by this function
    has x increasing with each column and y increasing with each consecutive
    row. Therefore, you index it like "grid[y,x]" instead of "grid[x,y]."

    Args:
        occupancy_grid (OccupancyGrid): Occupancy grid instance that the 2D
        array will be derived from.

    Returns:
        np.ndarray: The 2D numpy array constructed from the data of the provided
        occupancy grid.
    """
    return np.reshape(list(occupancy_grid.data), newshape=(occupancy_grid.info.height, occupancy_grid.info.width))
    # return np.array(occupancy_grid.data, dtype=np.int8).reshape((occupancy_grid.info.height, occupancy_grid.info.width))

def occupancy_grid_from_twod_numpy(twod_numpy_array: np.ndarray) -> List[int]:
    """Returns the row-major 1D array that resultings from flattening the
    provided 2D numpy array. I.e., the form of the 2D array the Occupancy Grid
    message type is expecting.

    Args:
        twod_numpy_array (np.ndarray): The 2D numpy array containing your
        occupancy grid data.

    Returns:
        List[int]: The row-major formatted list of integers derived from
        flattening the provided 2D array.
    """
    return twod_numpy_array.flatten().tolist()

def get_cell_cost() -> int:
    """Temporary function for returning the hardcoded cost value. In the future,
    this would likely be updated to incorporate some sort of probability based
    calculation.

    Returns:
        int: Cost probability as an integer between 0-->100.
    """
    return 100

def plot_coords_on_occupancy_grid(cell_coordinates: np.ndarray, occupancy_grid: OccupancyGrid) -> None:

    # 1. Filter out cell coordinates/indices that are out of bounds.
    filtered_cell_coords = filter_grid_coordinates(cell_coordinates=cell_coordinates,
                                                   grid_width=occupancy_grid.info.width,
                                                   grid_height=occupancy_grid.info.height)
    
    # 2. Index into the occupancy grid's data and set the cost value at each
    #    valid location from above. NOTE that for now, we're just
    #    unconditionally setting the cost to 100 to start.
    numpy_occupancy_grid = twod_numpy_from_occupancy_grid(occupancy_grid=occupancy_grid)
    numpy_occupancy_grid[filtered_cell_coords[:,1], filtered_cell_coords[:,0]] = get_cell_cost()
    occupancy_grid.data = occupancy_grid_from_twod_numpy(numpy_occupancy_grid)

def get_splat_radius(car_width: float, map_resolution_m_c: float) -> int:
    return 0.5*car_width*(1.0/map_resolution_m_c)

def get_splat_cells(splat_point: Tuple[int, int], splat_radius_c: int) -> np.ndarray:
    """Generates the "splat" filter centered at a particular point. Returns the
    (x,y) coordinate for every cell around the splat_point within the specified
    splat_radius. 

    Args:
        splat_point (Point): The (x,y) cell location of the point you want to
        splat.
        splat_radius_c (int): The splat radius (in number of cells).

    Returns:
        np.ndarray: The 2D array containing all the coordinates of the
        surrounding splat points.
    """
    splat_cell_offsets = []
    for x in range(splat_point[0] + splat_radius_c, splat_point[0] - splat_radius_c - 1, -1):
        for y in range(splat_point[1] + splat_radius_c, splat_point[1] - splat_radius_c - 1, -1):
            splat_cell_offsets.append([x,y])
    return splat_cell_offsets

# More optimized version of the above function but with fewer safeguards and
# individually tested functions. I.e., lower SIL code.
def laser_update_occupancy_grid_temp(scan_message: LaserScan,
                                     current_occupancy_grid: OccupancyGrid,
                                     splat_radius) -> OccupancyGrid:
    
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
    
    # Do the numpy equivalent of zipping together the pairs of x and y
    # coordinates.
    cell_coords = np.stack(arrays=[x_cell_coords, y_cell_coords], axis=1)
    # Splatting is going to be VERY expensive--like O(n^3). I think we'd be much
    # better off sliding a guassian filter across and just thresholding. Adjust
    # the standard deviation (width) of the guassian distribution to determine
    # how much it inflates obstacles. 

    # 1. Filter out cell coordinates/indices that are out of bounds.
    filtered_cell_coords = cell_coords[(cell_coords[:, 0] <= current_occupancy_grid.info.width-1)*(cell_coords[:, 0] >= 0)*\
                                       (cell_coords[:, 1] <= current_occupancy_grid.info.height-1)*(cell_coords[:, 1] >= 0)]
    
    # 2. Index into the occupancy grid's data and set the cost value at each
    #    valid location from above. NOTE that for now, we're just
    #    unconditionally setting the cost to 100 to start.
    # numpy_occupancy_grid = np.reshape(list(current_occupancy_grid.data), newshape=(current_occupancy_grid.info.height, current_occupancy_grid.info.width))
    numpy_occupancy_grid = np.zeros(shape=(current_occupancy_grid.info.height, current_occupancy_grid.info.width), dtype=np.int8)
    # numpy_occupancy_grid[filtered_cell_coords[:,1], filtered_cell_coor    ds[:,0]] = 100
    for cell_coords in filtered_cell_coords:
        numpy_occupancy_grid[cell_coords[1], cell_coords[0]] = 100
        # Additional points to apply splat. Not very robust/clean, but fast.
        # Determine the coordinates of the "splat" rectangle that surrounds the
        # point we'd like to splat at.
        x = cell_coords[0]
        y = cell_coords[1]
        y_min = max(0, min(y-splat_radius, current_occupancy_grid.info.height))
        y_max = max(0, min(y+splat_radius, current_occupancy_grid.info.height))
        x_min = max(0, min(x-splat_radius, current_occupancy_grid.info.width))
        x_max = max(0, min(x+splat_radius, current_occupancy_grid.info.width))
        # "Select" all the points within the splat rectangle and set their cost.
        numpy_occupancy_grid[y_min:y_max, x_min:x_max] = 100
        # TODO: Look up a way to just add a hardcoded filter centered somewhere
        # on a larger 2D numpy array. This is basically just the same question
        # as "how do I draw a circle or a square centered at a point in an
        # image" for opencv. This is not a hard problem.
    current_occupancy_grid.data = numpy_occupancy_grid.flatten().tolist()

    # Return updated current occupancy grid.
    return current_occupancy_grid
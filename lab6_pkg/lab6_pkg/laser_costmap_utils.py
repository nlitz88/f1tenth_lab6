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

# This is going to be a QUICK implementation of the more robust implementation
# above. If this works, I can re-implement this in a cleaner way (I.e., breaking
# things up like above).
def laser_update_occupancy_grid_temp(scan_message: LaserScan,
                                     current_occupancy_grid: OccupancyGrid,
                                     splat_radius,
                                     logger) -> OccupancyGrid:
    

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
    splat_radius = 4
    splat_coords = []
    for cell_coord in cell_coords:
        splat_points = get_splat_cells(cell_coord, splat_radius)
        splat_coords += splat_points
    cell_coords = np.array(list(cell_coords.tolist()) + splat_coords)
    
    # Rather than calling the below functions, just going to do things in here
    # for speed. No new cell coordinates should be introduced beyond this point.
    # THe process below is strictly for filtering and plotting.

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
    current_occupancy_grid.data = numpy_occupancy_grid.flatten().tolist()
    
    # plot_coords_on_occupancy_grid(cell_coordinates=cell_coords,
    #                               occupancy_grid=current_occupancy_grid)

    # Return updated current occupancy grid.
    return current_occupancy_grid
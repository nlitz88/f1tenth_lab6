
from typing import List, Tuple, Optional
import numpy as np
from collections import deque

from lab6_pkg.laser_costmap_utils import GridPosition

class Tree:
    """Tree implementation based on parallel arrays and adjacency lists.
    """
    def __init__(self, root_position: Tuple[int, int]):
        # Initialize the parallel adjacency lists and node_coordinates lists.
        self.__node_adjacency_lists: List[List[int]] = []
        self.__node_coordinates: List[Tuple[int,int]] = []
        # Add the root node to the tree.
        self.__node_adjacency_lists.append([])
        self.__node_coordinates.append(root_position)
        # Because this is the root node of the tree, the root isn't a child of
        # any other nodes. Therefore, we don't add it to any other node's
        # adjacency list.

    def __new_node(self, node_position: Tuple[int, int]) -> int:
        """Creates a new entry in the tree adjacency list for the new node with
        the provided position and returns its index. This index is what you'll
        use to access either its value in the parallel arrays adjacency list
        and/or node_coordinates.

        Args:
            node_position (Tuple[int, int]): The x,y position associated with
            this new node.

        Returns:
            int: The index of the new node to access both the parallel arrays
            that comprise the tree.
        """
        self.__node_adjacency_lists([])
        self.__node_coordinates.append(node_position)
        return len(self.__node_coordinates) - 1
    
    def __add_edge(self, parent_index: int, child_index: int) -> None:
        """Adds the child node's index to the parent node's adjacency list.

        Args:
            parent_index (int): Index of the parent node in the tree's adjacency
            list.
            child_index (int): Index of the child node in the tree's adjaceny
            list.
        """
        self.__node_adjacency_lists[parent_index].append(child_index)
        return

    def add_node(self, parent_index: int, new_node_position: Tuple[int, int]) -> int:
        """Creates a new node in the tree at the position specified as a child
        of the parent node specified.

        Args:
            parent_index (int): Index of the parent node in the tree's adjacency
            list.
            new_node_position (Tuple[int, int]): The x,y position associated
            with the new node.

        Returns:
            int: The index of the new node to access both the parallel arrays
            that comprise the tree.
        """
        new_node_index = self.__new_node(node_position=new_node_position)
        self.__add_edge(parent_index=parent_index, child_index=new_node_index)
        return new_node_index

    def get_node_coordinates(self, node_index: int) -> Tuple[int, int]:
        """Returns the coordinates of the single node with the provided
        node_index.

        Args:
            node_index (int): The index of the node in the tree.

        Returns:
            Tuple[int, int]: The x,y coordinate grid position of the node.
        """
        return self.__node_coordinates[node_index]

    def get_node_coordinates_array(self) -> np.ndarray:
        """A convenience function to return the internal node coordinates list
        as a 2D numpy array.

        Returns:
            np.ndarray: The 2D numpy array where each row is a different node's
            x,y coordinate position.
        """
        return np.array(self.__node_coordinates)

def free_space_from_costmap(costmap: np.ndarray,
                            occupied_threshold: Optional[int] = 100) -> np.ndarray:
    """Returns a 2D array (which could really be interpretted as a list of
    tuples) of all the free space coordinates found in the costmap.

    Args:
        costmap (np.ndarray): 2D numpy array of the costmap occupancy grid.
        occupied_threshold (Optional[int], optional): The cost that a cell must
        be in order to be considered occupied space. Defaults to 100.

    Returns:
        np.ndarray: The 2D array, which can be thought of as a list of tuples
        essentially, where each tuple is the x,y coordinates of a free space in
        the provided costmap.
    """
    return np.transpose(np.flip(np.where(costmap < occupied_threshold)))

def sample(free_space: np.ndarray) -> Tuple[int, int]:
    """Returns the coordinates of a random point in the unoccupied space of the
    provided 2D costmap. Throws an exception if there are no free spaces to
    sample from.

    Args:
        free_space (np.ndarray): A 2D array that contains (intuitively) a list
        of all the coordinates in the costmap that are not occupied.

    Raises:
        Exception: Raises exception if there are no free spaces for it to sample
        from.

    Returns:
        Tuple[int, int]: Returns the coordinates of the sampled space as a tuple
        (x,y).
    """
    # 1. Check if there are any free spaces to sample from.
    if free_space.shape[0] == 0:
        raise Exception("Free space sampling failed--no free spaces found to sample from in costmap!")
    # 2. Choose a random number between 0 and the length of the free coordinates
    #    list. Use a uniform distribution, giving equal probability of picking
    #    all points in free space.
    random_index = np.random.randint(low=0, high=free_space.shape[0])
    # 3. Return the coordinates at that index.
    return tuple(free_space[random_index])

def nearest_coord(node_coordinates: np.ndarray, sampled_point: np.ndarray) -> int:
    """Returns the index of the node in the node_coordinates array with the
    smallest euclidean distance.

    Args:
        node_coordinates (np.ndarray): 2D array where each row is a different
        node's x,y coordinates.
        sampled_point (np.ndarray): The x,y coordinates of the sampled point in
        the grid.

    Returns:
        int: The index of the node in node_coordinates with the smallest
        euclidean distance to the sampled_point.
    """
    return np.argmin(np.linalg.norm(node_coordinates-sampled_point))

def nearest(tree: Tree, sampled_point: Tuple[int, int]) -> int:
    """Returns the index of the node in the provided tree whose grid position is
    nearest to the provided sampled point.

    Args:
        tree (Tree): Tree of grid positions.
        sampled_point (Tuple[int, int]): The grid coordinates of the sampled
        point.

    Returns:
        int: The index of the node whose grid position is closest to the sampled
        point.
    """
    node_coordinates = tree.get_node_coordinates_array()
    nearest_node_index = nearest_coord(node_coordinates=node_coordinates,
                                       sampled_point=sampled_point)
    return nearest_node_index

def steer_and_check(nearest_point: Tuple[int, int],
                    sampled_point: Tuple[int, int],
                    costmap: np.ndarray,
                    new_point_distance: float) -> Tuple[int, int]:
    
    # Draw a line from nearest point from sampled point using Bresenham's line
    # algorithm. Walk along the the points in that line and stop if there is an
    # occupied cell along the line. If an occupied cell is not encountered.
    pass
    # Could also implement this between those two functions where I just first
    # generate what the new point must be using the angle and specified
    # distance.
    

    # Then, in the check collision function, I could 


def steer(nearest_point: Tuple[int, int],
          sampled_point: Tuple[int, int],
          new_point_distance: float,
          logger) -> Tuple[int, int]:
    """Returns the grid coordinates of a new point at new_point_distance from
    the nearest_point in the direction of the line from nearest_point to
    sampled_point.

    Args:
        nearest_point (Tuple[int, int]): nearest node on the tree to the sampled point
        sampled_point (Tuple[int, int]): sampled point
    Returns:
        Tuple[int, int]: The grid coordinates of the new node.
    """
    # Compute the angle of the line from the x-axis that separates these two
    # points. Need the resulting vector separating the two points, compute the
    # components of that vector first.
    x_comp = sampled_point[0] - nearest_point[0]
    y_comp = sampled_point[1] - nearest_point[1]
    logger.info(f"Vector x: {x_comp}, y: {y_comp}")
    # Compute the angle using an arctangent function that takes into account
    # different the different quadrants that our point x_comp, y_comp could fall
    # into.
    angle = np.arctan2(y_comp, x_comp)
    logger.info(f"Computed angle from x-axis: {angle} rad")
    # Compute the x and y component of the new point that is new_point_distance
    # from the starting point at the computed angle.
    x_new = int(nearest_point[0] + new_point_distance*np.cos(angle))
    y_new = int(nearest_point[1] + new_point_distance*np.sin(angle))
    return (x_new, y_new)

def point_in_costmap(point: Tuple[int, int], costmap: np.ndarray) -> bool:
    pass

def check_collision(nearest_point: Tuple[int, int],
                    new_point: Tuple[int, int], 
                    costmap: np.ndarray,
                    logger,
                    occupied_threshold: Optional[int] = 100) -> bool:
    """Check for a collision along the line connecting the nearest point to the
    new_point. Uses Bresenham's line algorithm to determine each x,y coordinate
    along the line and checks for occupancy at each position along the line.
    Returns false if no collision is found, returns true if occupied spaces are
    found along the path.

    Args:
        nearest_point (Tuple[int, int]): Point the line will drawn from. This
        point MUST be within the bounds of the costmap, will throw an exception
        otherwise.
        new_point (Tuple[int, int]): Point the line will be drawn to. This
        point MUST be within the bounds of the costmap, will throw an exception
        otherwise.
        costmap (np.ndarray): The 2D occupancy grid used to determine occupancy
        of spaces along the line.
        occupied_threshold (Optional[int], optional): The cost that a cell must
        be in order to be considered occupied space. Defaults to 100.

    Returns:
        bool: Returns True if a point along the path is found to be occupied in
        the occupancy grid, False if no points along the path are occupied.
    """
    collision_found = False
    # Extract the x and y components of the nearest point (which is our first
    # point), and the new_point (which is our second, destination point).
    x1, y1 = nearest_point
    x2, y2 = new_point
    # Using a version of Bresenham's algorithm that doesn't use floating point
    # derived from this GeeksForGeeks page:
    # https://www.geeksforgeeks.org/bresenhams-line-generation-algorithm/

    # NOTE: To account for the difference of these two points fallin in any
    # quadrant: Basically, we will always "walk the line" in the "first
    # quadrant," but based on the difference in x and y between the points, we
    # will "interpret" (negate or not) the points on the line drawn in the first
    # quadrant as being in the second, third, or fourth depending on the sign of
    # our differences. Determine those sign/polarity transforms here.
    # NOTE: UPDATE: So, after seeing this fail again, I think I realize why: We
    # don't want to INVERT the x and y values, as within the grid, we ONLY HAVE
    # POSITIVE INDICES! Therefore, instead of inverting, need to maintain the
    # sign of the offset. OR rather, just record whatever the offset is, sign
    # and everything, and then when we set the value of x2,y2 below to the
    # positive offset from the first point, we just add the absolute value of
    # that offset. Then, to get the "real value", add 2*the actual offset to the
    # x and y values.
    x_offset = x2-x1
    y_offset = y2-y1

    # Additionally, in order to make sure, then, that we're ALWAYS only drawing
    # in the "first quadrant," I think we just have to make sure our second
    # point is always the at the positive difference away from the starting
    # point. I.e., the starting point should ALWAYS remain the same. But from
    # this point forward, we should basically redefine x2,y2 (the second point)
    # to be equal to be the positive difference between the two offset from the
    # starting point.
    x2 = x1 + np.abs(x_offset)
    y2 = y1 + np.abs(y_offset)

    # Define variables to track current position, initialize at starting
    # position.
    y = y1
    x = x1

    # Have to evaluate the first position before any others.
    if x_offset < 0:
        x_real = x - 2*(x-x1)
    else:
        x_real = x
    if y_offset < 0:
        y_real = y - 2*(y-y1)
    else:
        y_real = y
    if costmap[y_real, x_real] >= occupied_threshold:
        logger.info(f"Found collision at point ({x_real},{y_real}) while walking from starting point {nearest_point} to ending point {new_point}")
        collision_found = True
    # Compute the adjusted slope (adjusted to avoid floating point arithmetic
    # later) and initialize the (adjusted) slope error.
    adjusted_slope = 2*(y2-y1)
    adjusted_slope_error = adjusted_slope - (x2-x1)

    while not collision_found and x < x2+1:
        
        # 1. Update the slope error for computing the next value. I.e., this
        #    will be the error once we'll use to figure out what to do to get
        #    the next value.
        adjusted_slope_error += adjusted_slope
        
        # 2. Get the next point in the line (know the next x value in the range,
        #    use Bresenham's to get the next corresponding y value).
        x += 1
        # If the slope error has reached 0, that means it's time to increment y
        # up to the next value.
        if adjusted_slope_error >= 0:
            y += 1
            adjusted_slope_error = adjusted_slope_error - 2 * (x2-x1)
        
        # 3. Interpret the "first quadrant line value" to the actual value based
        #    on the coordinate sign variables above.
        if x_offset < 0:
            x_real = x - 2*(x-x1)
        else:
            x_real = x
        if y_offset < 0:
            y_real = y - 2*(y-y1)
        else:
            y_real = y
        # 4. Check for collision at the interpretted position.
        if costmap[y_real, x_real] >= occupied_threshold:
            logger.info(f"Found collision at point ({x_real},{y_real}) while walking from starting point {nearest_point} to ending point {new_point}")
            collision_found = True

        # TODO AS A DEBUGGING STEP, I'm going to fill in each position on the
        # line.
        costmap[y_real,x_real] = 100

    return collision_found


def is_goal(self, latest_added_node, goal_x, goal_y):
    # IMPLEMENT THIS NEXT. Rename to "in goal region" or something like that,
    # but is_goal works, too.
    """
    This method should return whether the latest added node is close enough
    to the goal.

    Args:
        latest_added_node (Node): latest added node on the tree
        goal_x (double): x coordinate of the current goal
        goal_y (double): y coordinate of the current goal
    Returns:
        close_enough (bool): true if node is close enoughg to the goal
    """
    return False

def find_path(self, tree, latest_added_node):
    # This is where we'll backtrace through the graph.
    """
    This method returns a path as a list of Nodes connecting the starting point to
    the goal once the latest added node is close enough to the goal

    Args:
        tree ([]): current tree as a list of Nodes
        latest_added_node (Node): latest added node in the tree
    Returns:
        path ([]): valid path as a list of Nodes
    """
    path = []
    return path



# NOT SURE YET if this function specifically should return that Path object, or
# if it just should be a raw array of waypoints along the path--and then a
# separate function upstream could convert those into Poses and then into a
# path. Only say that because the function that calls this one will have all
# that metadata needed to construct a formal message.

# On that note: I think I want to implement this RRT function as being use-case
# agnostic. I.e., I want this algorithm to work no matter where it is deployed.
# I.e., this function shouldn't know anything about ROS--it should just be its
# own independent function that we call FROM the rrt node to run RRT on a
# provided 2D array/costmap--but nothing else.
def rrt(costmap: np.ndarray,
        start_point: GridPosition,
        goal_point: GridPosition,
        goal_radius: int,
        
        max_iterations: int,
        logger) -> List[GridPosition]:
    # Start point and goal point are both assumed to be within the bounds of the
    # provided costmap

    # INITITIALIZATION STEPS
    rrt_tree = Tree(root_position=(start_point.x, start_point.y))

    # I also think that this function should be the one responsible for
    # extracting that free space array from the costmap internally, as its not
    # needed outside of the context of this function! It's only ever needed in
    # here, therefore, I think that should be one of this function's
    # "initialization" steps.
    free_space = free_space_from_costmap(costmap=costmap)

    # RRT LOOP STEPS
    # Loop for a finite number of times or until a path has been found to the
    # goal region.
    path_found = False
    iteration_count = 0
    while iteration_count < max_iterations and not path_found:
        
        # 1. Randomly sample a point in free space.
        sampled_point_coords = sample(free_space=free_space)
        # 2. Find the point in the tree that is closest to the sampled point.
        nearest_node_to_sample = nearest(tree=rrt_tree, sampled_point=sampled_point_coords)
        nearest_point_coords = rrt_tree.get_node_coordinates(node_index=nearest_node_to_sample)
        # NOTE: DEBUG
        logger.info(f"Nearest point: {(rrt_tree.get_node_coordinates(node_index=nearest_node_to_sample))}")

        # Check for a collision along the line from the nearest point in the
        # tree to the sampled point to determine if we can add a new node in
        # the direction of that node.
        # NOTE: as an optimization for later, we could instead check to see if
        # there is a collision within some fixed distance. If there isn't a
        # collision within a certain distance (that we could perhaps check first
        # with the underlying function), then we can just make the new node at
        # that distance along that line. For the sake of simplicity and time,
        # I think I may just do this to start. I.e., sorta morphing the check
        # collision and new point in one. But really, we're just checking for a
        # collision within the distance that we'd make that new node at.

        # TODO: Need to call a function here that checks to see if the
        # point/grid position returned by steer is actually within the bounds of
        # the costmap.


        # DEBUG
        path_found = True

    return []
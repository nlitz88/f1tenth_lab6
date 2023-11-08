
from typing import List, Tuple, Optional
import numpy as np
from collections import deque

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
    
    def backtrace(self, start_node_index: int, goal_node_index: int) -> List[int]:
        # TODO: Implement this function to iterate through the graph and return
        # the path to the goal node.
        pass

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

    Returns:
        Tuple[int, int]: Returns the coordinates of the sampled space as a tuple
        (x,y).
    """
    # 1. Choose a random number between 0 and the length of the free coordinates
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
          new_point_distance: int) -> Tuple[int, int]:
    """Returns the grid coordinates of a new point at new_point_distance from
    the nearest_point in the direction of the line from nearest_point to
    sampled_point.

    Args:
        nearest_point (Tuple[int, int]): nearest node on the tree to the sampled point
        sampled_point (Tuple[int, int]): sampled point
        new_point_distance (int): The distance (in cells) from the nearest node
        the new node should be placed (along the line from nearest to sampled). 
    Returns:
        Tuple[int, int]: The grid coordinates of the new node.
    """
    # Compute the angle of the line from the x-axis that separates these two
    # points. Need the resulting vector separating the two points, compute the
    # components of that vector first.
    x_comp = sampled_point[0] - nearest_point[0]
    y_comp = sampled_point[1] - nearest_point[1]
    # Compute the angle using an arctangent function that takes into account
    # different the different quadrants that our point x_comp, y_comp could fall
    # into.
    angle = np.arctan2(y_comp, x_comp)
    # Compute the x and y component of the new point that is new_point_distance
    # from the starting point at the computed angle.
    x_new = int(nearest_point[0] + new_point_distance*np.cos(angle))
    y_new = int(nearest_point[1] + new_point_distance*np.sin(angle))
    return (x_new, y_new)

def point_in_costmap(point: Tuple[int, int], costmap: np.ndarray) -> bool:
    """Returns True if the provided point falls within the boundaries of the
    costmap, False if not.

    Args:
        point (Tuple[int, int]): The x,y grid position that will be checked. 
        costmap (np.ndarray): The 2D occupancy grid we'll be checking if point
        is in or not. This function is expecting that the occupancy grid is laid
        out such that each row corresponds with a different y value, each column
        corresponds to a different x value.

    Returns:
        bool: True if the point is within the bounds of the costmap, False if
        not.
    """
    point_x, point_y = point
    min_x = 0
    max_x = costmap.shape[1] - 1
    min_y = 0
    max_y = costmap.shape[0] - 1
    # If the point's x coordinate is out of bounds.
    if point_x < min_x or point_x > max_x:
        return False
    # If the point's y coordinate is out of bounds.
    if point_y < min_y or point_y > max_y:
        return False
    # If neither of the above conditions is violated, then the point must be in
    # bounds--return True.
    return True

def collision(nearest_point: Tuple[int, int],
             new_point: Tuple[int, int], 
             costmap: np.ndarray,
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

    # Problem: This algorithm doesn't allow us to draw horizontal lines (or
    # lines along the y-axis). How do we resolve this?
    # WELL, the bresenham line algorithm really only works for lines between 0
    # and 45 degrees. Anything more than that and you need to apply some
    # creative logic to use it for other angles.
    # Per a stack overflow answer: the best way to tackle this is to identify:
    # if the change in y is greater than the change in x (I.e., if the point
    # you're trying to draw a line to exceeds that 45 degree mark), then instead
    # of incrementing over x and computing each y, we have to increment over
    # each y and compute each x.

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

    # Change which version of the algorithm is used based on which direction's
    # offset is larger.
    if np.abs(x_offset) >= np.abs(y_offset):

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
                collision_found = True

    else:
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
            collision_found = True
        # Compute the adjusted slope (adjusted to avoid floating point arithmetic
        # later) and initialize the (adjusted) slope error.
        adjusted_slope = 2*(x2-x1)
        adjusted_slope_error = adjusted_slope - (y2-y1)

        while not collision_found and y < y2+1:
            
            # 1. Update the slope error for computing the next value. I.e., this
            #    will be the error once we'll use to figure out what to do to get
            #    the next value.
            adjusted_slope_error += adjusted_slope
            
            # 2. Get the next point in the line (know the next x value in the range,
            #    use Bresenham's to get the next corresponding y value).
            y += 1
            # If the slope error has reached 0, that means it's time to increment y
            # up to the next value.
            if adjusted_slope_error >= 0:
                x += 1
                adjusted_slope_error = adjusted_slope_error - 2 * (y2-y1)
            
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
                collision_found = True

    return collision_found

def in_goal_region(new_point: Tuple[int, int],
                   goal_point: Tuple[int, int],
                   goal_radius_c: int) -> bool:
    """Returns whether the provided new_point falls within the goal region,
    making it close enough to consider being at the goal point.

    Args:
        new_point (Tuple[int, int]): The x,y grid position being evaluated
        against the goal region.
        goal_point (Tuple[int, int]): The x,y grid position of the goal
        position. 
        goal_radius_c (int): The radius (in # of cells) that the new_point must
        approximately be within in order to be considered within the goal
        region.

    Returns:
        bool: True if the new point is close enough to the goal (I.e., it falls
        within goal_radius_c of the goal_point), False if not.
    """

    # Compute the boundary values for the goal square using the goal point and
    # the goal radius.
    goal_point_x, goal_point_y = goal_point
    # NOTE: The goal square's boundary values CAN fall outside of the costmap,
    # as this function only cares about proximity. I.e., out of bounds
    # coordinates are handled outside the scope of this function.
    goal_square_x_max = goal_point_x + goal_radius_c
    goal_square_x_min = goal_point_x - goal_radius_c
    goal_square_y_max = goal_point_y + goal_radius_c
    goal_square_y_min = goal_point_y - goal_radius_c

    # Determine if the new_point falls within the goal_square. Or, rather,
    # return False if either of the new point coordinates fall outside of their
    # respective boundaries, return True if both coordinates are within the goal
    # square boundaries. 
    new_point_x, new_point_y = new_point
    if new_point_x < goal_square_x_min or new_point_x > goal_square_x_max:
        return False
    if new_point_y < goal_square_y_min or new_point_y > goal_square_y_max:
        return False
    return True

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
        start_point_coords: Tuple[int, int],
        goal_point_coords: Tuple[int, int],
        goal_radius_c: int,
        new_point_distance: int,
        max_iterations: int) -> List[Tuple[int, int]]:
    # Start point and goal point are both assumed to be within the bounds of the
    # provided costmap

    # INITITIALIZATION STEPS
    # Create a new tree instance and initialize it with the root equal to the
    # starting point.
    rrt_tree = Tree(root_position=start_point_coords)

    # I also think that this function should be the one responsible for
    # extracting that free space array from the costmap internally, as its not
    # needed outside of the context of this function! It's only ever needed in
    # here, therefore, I think that should be one of this function's
    # "initialization" steps.
    free_space = free_space_from_costmap(costmap=costmap)
    
    # Variables to control iteration and how we backtrace our path.
    goal_reached = False
    root_node_index = 0
    goal_node_index = root_node_index

    # Before running any RRT iterations, make sure there is free space to sample
    # from. If not, just set the goal as reached, and set the goal node to the
    # root node.
    if free_space.shape[0] == 0:
        # Provide some sort of warning??
        goal_reached = True

    # RRT LOOP STEPS
    # Loop for a finite number of times or until a path to the goal region has
    # been found.
    iteration_count = 0
    while iteration_count < max_iterations and not goal_reached:
        
        # 1. Randomly sample a point in free space.
        sampled_point_coords = sample(free_space=free_space)
        # 2. Find the point in the tree that is closest to the sampled point.
        nearest_node_to_sample = nearest(tree=rrt_tree, sampled_point=sampled_point_coords)
        nearest_point_coords = rrt_tree.get_node_coordinates(node_index=nearest_node_to_sample)
        # 3. Use the steer function to determine the location of a node along
        #    the line from from the nearest tree node to the sampled node, some
        #    distance new_node_distance away from the nearest tree node.
        new_point_coords = steer(nearest_point=nearest_point_coords,
                                sampled_point=sampled_point_coords,
                                new_point_distance=new_point_distance)
        # 4. Check to see if the new node's coordinates are within the costmap's
        #    bounds. If it's not within the costmap, then just skip over this
        #    point
        if not point_in_costmap(point=new_point_coords, costmap=costmap):
            continue
    
        # 5. If it is within the costmap, also check to see if there are any
        #    abstacles along the line from the nearest node to this newest node,
        #    or if the newest node just falls on top of occupied space. If there
        #    is a collision, just move on and skip over this point.
        if collision(nearest_point=nearest_point_coords,
                     new_point=new_point_coords,
                     costmap=costmap):
            continue

        # 6. If the point is within the costmap and there aren't any obstacles
        #    in between it and the nearest node, then we can add it to the tree
        #    as the child of teh nearest node.
        new_node_index = rrt_tree.add_node(parent_index=nearest_node_to_sample)

        # 7. Finally, check to see if the point falls within the goal region. If
        #    so, then the new_node_index is our goal node, the path will be
        #    backtraced for this new node, and RRT has finished planning a path.
        if in_goal_region(new_point=new_point_coords,
                          goal_point=goal_point_coords,
                          goal_radius_c=goal_radius_c):
            goal_reached = True
            goal_node_index = new_node_index

    # In either case below, we're going to return a path, which will start as a
    # list of Tuple x,y positions.
    path = List[Tuple[int, int]]

    # If the goal node HAS BEEN REACHED, then call the backtrace function on the
    # tree to get the path from the root of the tree (the starting position) to
    # the goal node (the point in the goal region).
    if goal_reached:
        pass

    # If the goal HAS NOT been reached by the end of the iterations, I think the
    # safest action to take is to just find the node in the tree that is CLOSEST
    # to the goal point, select that closest node as being the goal node, and
    # then just publishing a path from the start to that goal node, even if it
    # hasn't truly reached the end. This way, your path tracker still has a path
    # to track, but it's a safe path that it should be able to observe the end
    # of and hopefully stop in time.

    else:
        pass

    
    # Then, from either of the above, return the list of Tuple positions.


    return []
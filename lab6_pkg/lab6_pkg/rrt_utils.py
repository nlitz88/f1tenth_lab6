
from typing import Tuple, Optional
import numpy as np

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
        essentially, where each tuple is the X,Y coordinates of a free space in
        the provided costmap.
    """
    return np.transpose(np.flip(np.where(costmap < occupied_threshold)))

# def sample(costmap: np.ndarray,
#            occupied_threshold: Optional[int] = 100) -> Tuple[int, int]:
#     """Returns the coordinates of a random point in the unoccupied space of the
#     provided 2D costmap. Throws an exception if there are no free spaces to
#     sample from.

#     Args:
#         costmap (np.ndarray): 2D array derived from an a costmap occupancy grid.
#         occupied_threshold (Optional[int], optional): The cost that a cell must be
#         in order to be considered occupied space. Defaults to 100.

#     Raises:
#         Exception: Raises exception if there are no free spaces for it to sample
#         from.

#     Returns:
#         Tuple[int, int]: Returns the coordinates of the sampled space as a tuple
#         (x,y).
#     """
#     # 1. Use np.where to get the x and y coordinates of the locations where the
#     #    cost is less than the cost threshold. Take the transpose to get an
#     #    array of [x,y] arrays, rather than two separate, parallel x and y index
#     #    arrays. Have to flip so that x coords (columns) are on top.
#     # TODO: May want to separate this out into a separate block or something and
#     # pass it in, as we're going to have to sample from this free_coords array
#     # repeatedly.
    
#     if free_coords.shape[0] == 0:
#         raise Exception("No free spaces found to sample from in costmap.")
#     # 2. Choose a random number between 0 and the length of the free coordinates
#     #    list. Use a uniform distribution, giving equal probability of picking
#     #    all points in free space.
#     random_index = np.random.randint(low=0, high=free_coords.shape[0])
#     # 3. Return the coordinates at that index.
#     return tuple(free_coords[random_index])

def nearest(self, tree, sampled_point):
    """
    This method should return the nearest node on the tree to the sampled point

    Args:
        tree ([]): the current RRT tree
        sampled_point (tuple of (float, float)): point sampled in free space
    Returns:
        nearest_node (int): index of neareset node on the tree
    """
    nearest_node = 0
    return nearest_node

def steer(self, nearest_node, sampled_point):
    """
    This method should return a point in the viable set such that it is closer 
    to the nearest_node than sampled_point is.

    Args:
        nearest_node (Node): nearest node on the tree to the sampled point
        sampled_point (tuple of (float, float)): sampled point
    Returns:
        new_node (Node): new node created from steering
    """
    new_node = None
    return new_node

def check_collision(self, nearest_node, new_node):
    """
    This method should return whether the path between nearest and new_node is
    collision free.

    Args:
        nearest (Node): nearest node on the tree
        new_node (Node): new node from steering
    Returns:
        collision (bool): whether the path between the two nodes are in collision
                            with the occupancy grid
    """
    # NOTE: Can probably use Bresenham's line algorithm to help us with
    # this--probably the most straightforward.
    return True

def is_goal(self, latest_added_node, goal_x, goal_y):
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



# The following methods are needed for RRT* and not RRT
def cost(self, tree, node):
    """
    This method should return the cost of a node

    Args:
        node (Node): the current node the cost is calculated for
    Returns:
        cost (float): the cost value of the node
    """
    return 0

def line_cost(self, n1, n2):
    """
    This method should return the cost of the straight line between n1 and n2

    Args:
        n1 (Node): node at one end of the straight line
        n2 (Node): node at the other end of the straint line
    Returns:
        cost (float): the cost value of the line
    """
    return 0

def near(self, tree, node):
    """
    This method should return the neighborhood of nodes around the given node

    Args:
        tree ([]): current tree as a list of Nodes
        node (Node): current node we're finding neighbors for
    Returns:
        neighborhood ([]): neighborhood of nodes as a list of Nodes
    """
    neighborhood = []
    return neighborhood
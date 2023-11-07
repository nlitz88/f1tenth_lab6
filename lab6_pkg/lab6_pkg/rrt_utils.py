
from typing import List, Tuple, Optional
import numpy as np

from lab6_pkg.laser_costmap_utils import GridPosition

# This should be an application independent/agnostic tree adjacency list
# implementation.
class TreeNode:
    """Class describing each node of the RRT tree that we'll be constructing."""
    def __init__(self):
        self.x = None
        self.y = None
        self.children = []

# Need to decide if I'm going to store the tree as an array or as more of a
# linked list. Just have to remember: Every time we sample a new point, have to
# iterate through all the nodes of the tree to figure out which is closest.
# Which means, if we do the linked list style organization, that means we do a
# BFS over the entire tree every time, making it O(V+E). So, it's not that bad,
# but it is certainly not as nice as just doing a vectorized euclidean distance
# or something like that.

# Now, for the sake of simplicity and getting something working, I can always
# just implement it as the linked list style, as that's probably faster to
# implement. BUT, either way, I NEED to make some helper functions that abstract
# away manipulating the tree 

# What operations do I need functions for? 
# Inserting a new node (I.e., for a new point in the RRT tree).

# AND, Actually, I think I'm realizing that the tree we construct with RRT is
# NOT actually a binary tree, but instead a general tree. I.e., each node can
# have an abitrary number of children.

# Therefore, if the above is true, we can't use the "tree as array"
# implementation, as that only works if you have an n-ary tree, where you the
# maximum number of children each node can have.

# Therefore, I think a better representation of the graph would now be with an
# adjacency list. Because it's a tree (and not a generalized graph), we *could*
# use a parent point in each node object. That would prevent you from having to
# traverse up the graph, but it could be good. However, this isn't robust to
# nodes being removed. In our case, we really don't have to worry about that,
# but it would be nice to do this the right way if it's not that much hard.

# I think I can just implement a function to find the path from the end back to
# the start or something.

class Tree:
    def __init__(self):
        self.__adjacency_list = []
    

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

def nearest(tree_head: TreeNode, sampled_point):
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
        max_iterations: int) -> List[GridPosition]:
    # Start point and goal point are both assumed to be within the bounds of the
    # provided costmap


    # INITITIALIZATION STEPS

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
        
        # Randomly sample a point in free space.
        sampled_point = sample(free_space=free_space)

        # Find the point in the tree that is closest to the sampled point.
        nearest(tree="asdf", sampled_point=sampled_point)
    
    pass
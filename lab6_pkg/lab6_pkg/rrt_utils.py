
from typing import List, Tuple, Optional
import numpy as np
from collections import deque

from lab6_pkg.laser_costmap_utils import GridPosition

class Tree:
    """Tree implementation based on parallel arrays and adjacency lists.
    """
    def __init__(self):
        self.__node_adjacency_lists: List[List[int]] = []
        self.__node_coordinates: List[Tuple[int,int]] = []

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

        pass

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

    def get_node_coordinates(self) -> np.ndarray:
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
    node_coordinates = tree.get_node_coordinates()
    nearest_node_index = nearest_coord(node_coordinates=node_coordinates,
                                       sampled_point=sampled_point)
    return nearest_node_index



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

        # Get a list of all 

        # Find the point in the tree that is closest to the sampled point.
        nearest(tree="asdf", sampled_point=sampled_point)
    
    pass
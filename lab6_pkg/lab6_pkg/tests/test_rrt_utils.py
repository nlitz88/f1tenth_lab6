import unittest
from lab6_pkg.rrt_utils import *

class TestFreeSpaceFromCostmap(unittest.TestCase):
    def test_empty_costmap(self):
        costmap = np.array([])
        result = free_space_from_costmap(costmap)
        self.assertTrue(result.shape[0] == 0)

    def test_no_free_space(self):
        costmap = np.array([[200, 200, 200], [200, 200, 200], [200, 200, 200]])
        result = free_space_from_costmap(costmap)
        self.assertTrue(result.shape[0] == 0)

    def test_some_free_space(self):
        costmap = np.array([[50, 200, 50], [200, 50, 200], [50, 200, 50]])
        result = free_space_from_costmap(costmap)
        expected_result = np.flip(np.array([[0, 0], [0, 2], [1, 1], [2, 0], [2, 2]]))
        self.assertTrue(np.array_equal(result, expected_result))

    def test_custom_occupied_threshold(self):
        costmap = np.array([[90, 200, 90], 
                            [200, 90, 200], 
                            [90, 200, 90]])
        occupied_threshold = 100
        result = free_space_from_costmap(costmap, occupied_threshold)
        expected_result = np.flip(np.array([[0, 0], [0, 2], [1, 1], [2,0], [2, 2]]))
        self.assertTrue(np.array_equal(result, expected_result))

class TestSample(unittest.TestCase):
    def test_single_free_space(self):
        free_space = np.array([[1, 1]])
        result = sample(free_space)
        self.assertEqual(result, (1, 1))

    def test_multiple_free_space(self):
        free_space = np.array([[0, 0], [1, 1], [2, 2], [3, 3]])
        result = sample(free_space)
        self.assertTrue(result in [(0, 0), (1, 1), (2, 2), (3, 3)])

class TestPointInCostmap(unittest.TestCase):

    def test_point_within_costmap(self):
        # Create a 3x3 costmap for testing
        costmap = np.array([[0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0]])
        point = (1, 1)  # This point is within the costmap
        result = point_in_costmap(point, costmap)
        self.assertTrue(result)

    def test_point_outside_costmap(self):
        # Create a 3x3 costmap for testing
        costmap = np.array([[0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0]])
        point = (-1, 1)  # This point is outside the costmap
        result = point_in_costmap(point, costmap)
        self.assertFalse(result)

    def test_point_on_costmap_edge(self):
        # Create a 3x3 costmap for testing
        costmap = np.array([[0, 0, 0],
                            [0, 0, 0],
                            [0, 0, 0]])
        point = (0, 2)  # This point is on the costmap edge
        result = point_in_costmap(point, costmap)
        self.assertTrue(result)

class TestInGoalRegion(unittest.TestCase):

    def test_within_goal_region(self):
        goal_point = (5, 5)
        goal_radius = 3
        new_point = (6, 6)
        result = in_goal_region(new_point, goal_point, goal_radius)
        self.assertTrue(result)

    def test_outside_goal_region(self):
        goal_point = (5, 5)
        goal_radius = 3
        new_point = (10, 10)
        result = in_goal_region(new_point, goal_point, goal_radius)
        self.assertFalse(result)

    def test_on_goal_region_edge(self):
        goal_point = (5, 5)
        goal_radius = 3
        new_point = (5, 8)
        result = in_goal_region(new_point, goal_point, goal_radius)
        self.assertTrue(result)

class TestTreeBacktrace(unittest.TestCase):

    def test_backtrace_single_node(self):
        # Create a tree with a single root node.
        tree = Tree((0, 0))
        
        # The backtrace for the root node itself should be the root's coordinates.
        path = tree.backtrace(0)
        expected_path = [(0, 0)]
        
        self.assertEqual(path, expected_path)

    def test_backtrace_two_nodes(self):
        # Create a tree with two nodes.
        tree = Tree((0, 0))
        child_node_index = tree.add_node(0, (1, 1))
        
        # The backtrace for the child node should include both nodes' coordinates.
        path = tree.backtrace(child_node_index)
        expected_path = [(0, 0), (1, 1)]
        
        self.assertEqual(path, expected_path)

    def test_backtrace_complex_tree(self):
        # Create a more complex tree.
        tree = Tree((0, 0))
        child1_index = tree.add_node(0, (1, 1))
        child2_index = tree.add_node(0, (1, -1))
        child3_index = tree.add_node(child1_index, (2, 2))
        child4_index = tree.add_node(child2_index, (2, -2))
        
        # Test backtrace for child3_index, which should include coordinates from root to child3.
        path = tree.backtrace(child3_index)
        expected_path = [(0, 0), (1, 1), (2, 2)]
        
        self.assertEqual(path, expected_path)

    def test_backtrace_multiple_levels(self):
        # Create a tree with multiple levels.
        tree = Tree((0, 0))
        child1_index = tree.add_node(0, (1, 1))
        child2_index = tree.add_node(0, (1, -1))
        child3_index = tree.add_node(child1_index, (2, 2))
        child4_index = tree.add_node(child1_index, (2, 1))
        child5_index = tree.add_node(child2_index, (2, -2))
        
        # Test backtrace for child4_index, which should include coordinates from root to child4.
        path = tree.backtrace(child4_index)
        expected_path = [(0, 0), (1, 1), (2, 1)]
        
        self.assertEqual(path, expected_path)

    # def test_backtrace_goal_not_in_tree(self):
    #     # Create a tree with a single root node.
    #     tree = Tree((0, 0))
        
    #     # Try to backtrace to a node that is not in the tree (index 1).
    #     path = tree.backtrace(1)
        
    #     # The result should be an empty list since the goal node is not in the tree.
    #     expected_path = []
        
    #     self.assertEqual(path, expected_path)

    def test_backtrace_root_as_goal(self):
        # Create a tree with a single root node.
        tree = Tree((0, 0))
        
        # Try to backtrace to the root node itself (index 0).
        path = tree.backtrace(0)
        
        # The result should be a list containing only the root's coordinates.
        expected_path = [(0, 0)]
        
        self.assertEqual(path, expected_path)

if __name__ == "__main__":
    unittest.main()
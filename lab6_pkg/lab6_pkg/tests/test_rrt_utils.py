import unittest
from lab6_pkg.rrt_utils import *

# class TestSampleFunction(unittest.TestCase):

#     def test_fully_occupied(self):
#         costmap = np.array([[150, 150], [150, 150]]) # costmap with all cells occupied
#         with self.assertRaises(Exception):
#             sample(costmap)

#     def test_some_free_spaces(self):
#         costmap = np.array([[150, 50], [150, 150]]) # costmap with some free spaces
#         x, y = sample(costmap)
#         self.assertTrue(0 <= x < costmap.shape[0] and 0 <= y < costmap.shape[1])

#     def test_different_sizes(self):
#         costmap1 = np.array([[50, 50], [50, 50]]) # 2x2 costmap
#         costmap2 = np.array([[50, 50, 50], [50, 50, 50], [50, 50, 50]]) # 3x3 costmap
#         costmap3 = np.array([[50, 50, 50, 50], [50, 50, 50, 50], [50, 50, 50, 50], [50, 50, 50, 50]]) # 4x4 costmap
#         for costmap in [costmap1, costmap2, costmap3]:
#             x, y = sample(costmap)
#             self.assertTrue(0 <= x < costmap.shape[0] and 0 <= y < costmap.shape[1])

#     def test_one_free_space(self):
#         costmap = np.array([[150, 150], [50, 150]]) # costmap with one free space
#         x, y = sample(costmap)
#         self.assertTrue((x, y) == (1, 0))


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

if __name__ == "__main__":
    unittest.main()
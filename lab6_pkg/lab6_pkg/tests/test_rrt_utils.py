import unittest
from lab6_pkg.rrt_utils import *

class TestSampleFunction(unittest.TestCase):

    def test_fully_occupied(self):
        costmap = np.array([[150, 150], [150, 150]]) # costmap with all cells occupied
        with self.assertRaises(Exception):
            sample(costmap)

    def test_some_free_spaces(self):
        costmap = np.array([[150, 50], [150, 150]]) # costmap with some free spaces
        x, y = sample(costmap)
        self.assertTrue(0 <= x < costmap.shape[0] and 0 <= y < costmap.shape[1])

    def test_different_sizes(self):
        costmap1 = np.array([[50, 50], [50, 50]]) # 2x2 costmap
        costmap2 = np.array([[50, 50, 50], [50, 50, 50], [50, 50, 50]]) # 3x3 costmap
        costmap3 = np.array([[50, 50, 50, 50], [50, 50, 50, 50], [50, 50, 50, 50], [50, 50, 50, 50]]) # 4x4 costmap
        for costmap in [costmap1, costmap2, costmap3]:
            x, y = sample(costmap)
            self.assertTrue(0 <= x < costmap.shape[0] and 0 <= y < costmap.shape[1])

    def test_one_free_space(self):
        costmap = np.array([[150, 150], [50, 150]]) # costmap with one free space
        x, y = sample(costmap)
        self.assertTrue((x, y) == (1, 0))

if __name__ == "__main__":
    unittest.main()
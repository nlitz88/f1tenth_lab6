import unittest
from lab6_pkg.laser_costmap_utils import *

class TestPositionFromRange(unittest.TestCase):
    def test_position_from_range_positive_range(self):
        # Test with a range of 1 meter and an angle of 45 degrees (pi/4 radians).
        result = position_from_range(1.0, 0.25 * math.pi)
        self.assertAlmostEqual(result.x, 0.70710678118, places=5)  # Expected x value
        self.assertAlmostEqual(result.y, 0.70710678118, places=5)  # Expected y value

    def test_position_from_range_large_range(self):
        # Test with a range of 2 meters and an angle of 60 degrees (pi/3 radians).
        result = position_from_range(2.0, 1/3 * math.pi)
        self.assertAlmostEqual(result.x, 1.0, places=5)  # Expected x value
        self.assertAlmostEqual(result.y, 1.73205080757, places=5)  # Expected y value

    def test_position_from_range_zero_range(self):
        # Test with a range of 0 meters and an angle of 90 degrees (pi/2 radians).
        result = position_from_range(0.0, 0.5 * math.pi)
        self.assertAlmostEqual(result.x, 0.0, places=5)  # Expected x value
        self.assertAlmostEqual(result.y, 0.0, places=5)  # Expected y value

    def test_position_from_range_negative_angle(self):
        # Test with a range of 3 meters and a negative angle of -45 degrees (-pi/4 radians).
        result = position_from_range(3.0, -0.25 * math.pi)
        self.assertAlmostEqual(result.x, 3.0*0.70710678118, places=5)  # Expected x value
        self.assertAlmostEqual(result.y, 3.0*-0.70710678118, places=5)  # Expected y value

    def test_position_from_range_large_angle(self):
        # Test with a range of 1 meter and a large angle of 135 degrees (3*pi/4 radians).
        result = position_from_range(1.0, 0.75 * math.pi)
        self.assertAlmostEqual(result.x, -0.70710678118, places=5)  # Expected x value
        self.assertAlmostEqual(result.y, 0.70710678118, places=5)  # Expected y value


class TestProjectContinuousPointToGrid(unittest.TestCase):
    def test_point_at_origin(self):
        # Test for a point at the origin (0, 0)
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.resolution = 0.1
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        continuous_point = (0.0, 0.0)
        result = project_continuous_point_to_grid(occupancy_grid, continuous_point)
        self.assertEqual(result, (0, 0))

    def test_point_at_non_origin_position(self):
        # Test for a point at a non-origin position
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.resolution = 0.1
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        continuous_point = (0.5, 0.5)
        result = project_continuous_point_to_grid(occupancy_grid, continuous_point)
        self.assertEqual(result, (5, 5))  # 0.5 / 0.1 = 5

    def test_point_with_different_resolution_and_origin(self):
        # Test for a point with a different resolution and origin
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.resolution = 0.2
        occupancy_grid.info.origin.position.x = 1.0
        occupancy_grid.info.origin.position.y = 1.0
        continuous_point = (2.5, 2.5)
        result = project_continuous_point_to_grid(occupancy_grid, continuous_point)
        self.assertEqual(result, (7, 7))  # (2.5 - 1.0) / 0.2 = 7

if __name__ == "__main__":
    unittest.main()
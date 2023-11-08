
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Path

# NOTE: Many of these helper functions for choosing the next target point are
# taken DIRECTLY from my pure pursuit implementation, as it uses the same
# algorithm/process to select the next target point. I.e., in for our goal point
# publisher, we're effectively using the pure pursuit algorithm to choose which
# waypoint we're going to tell our local planner to create a path to next.

def numpy_array_from_path(path: Path) -> np.ndarray:
    """Takes a Path object and returns a 2D numpy array 

    Args:
        path (Path): Collection of PoseStamped pose that form a path.

    Returns:
        np.ndarray: A numpy array of 2D positions derived from the Poses of the
        provided path.
    """
    return np.array([[float(pose.pose.position.x), float(pose.pose.position.y)] for pose in path.poses])

def numpy_position_from_pose(pose: Pose) -> np.ndarray:
    """Helper function to extract the position from the provided pose as a 1D
    numpy ndarray. I.e., given a Pose, returns an ndarray like [x, y].

    Args:
        pose (Pose): Pose instance position will be extracted from.

    Returns:
        np.ndarray: 2D position extracted from provided pose as ndarray([x, y]).
    """
    return np.array([pose.position.x, pose.position.y])

def euclidean_distance(vector_a: np.ndarray, vector_b: np.ndarray) -> float:
    """Tiny helper function to compute euclidean distance using numpy.
    Fast method found on
    https://stackoverflow.com/questions/1401712/how-can-the-euclidean-distance-be-calculated-with-numpy
    
    Args:
        row (np.ndarray): numpy array row.

    Returns:
        float: The euclidean distance between vector_a and vector_b.
    """
    return np.linalg.norm(vector_a-vector_b)

def get_distance_to_each_point(current_position: np.ndarray, 
                               numpy_path: np.ndarray) -> np.ndarray:
    """Given your current 2D position in a given frame of reference and a numpy
    array of 2D positions in that same frame that describe a path (a sequence of
    waypoints), this function computes the distance from your current position
    to EACH of the nodes in the provided path. Returns a numpy array with as
    many distances as there are points (rows) in the input array numpy_path.
    I.e., the ith distance in the returned array will be the euclidean distance
    from the provided current_position to the ith waypoint in the input
    numpy_path array.

    Args:
        current_position (np.ndarray): Your current position (x,y) in a
        given frame as a 1D ndarray.
        numpy_path (np.ndarray): Array of 2D waypoints comprising a path in the
        same frame as the current_position.

    Returns:
        np.ndarray: The array of distances from your current position to each of
        the positions/waypoints in the provided numpy_path.
    """
    return np.apply_along_axis(euclidean_distance, 1, numpy_path, current_position)

def get_smallest_index(distances: np.ndarray) -> int:
    """Returns the index of the smallest distance in the provided array. Note
    that if there are identical elements that are both the smallest, this
    returns the index of the first instance of that smallest value.

    Args:
        distances (np.ndarray): The ndarray of distances the smallest will be
        found in.

    Returns:
        int: The index of the smallest distance found.
    """
    return np.argmin(distances)

def get_distances_starting_at_index(distances: np.ndarray, index: int) -> np.ndarray:
    """Returns a view (slice) of the provided distances starting at the provided
    index.

    Args:
        numpy_path (np.ndarray): The array of distances.
        index (int): The index that the slice/view will start with.

    Returns:
        np.ndarray: The array of elements in distances from the given index and
        beyond.
    """
    return distances[index:]

def normalize_distances(lookahead_distance_m: float, distances_m: np.ndarray) -> np.ndarray:
    """Subtracts the lookahead distance from each element in distances and then
    takes the absolute value. I.e., just a simple wrapper for element-wise
    subtraction and absolute value.

    Args:
        lookahead_distance_m (float): The distance to be subtracted from each
        distance in the provided distances array.
        distances_m (np.ndarray): The array of distances.

    Returns:
        np.ndarray: The "normalized" distances. I.e., the absolute value of the
        distances with the lookahead distance subtracted from each.
    """
    return np.abs(np.subtract(distances_m, lookahead_distance_m))

def get_next_target_point(current_pose: PoseWithCovarianceStamped, 
                          path: Path,
                          lookahead_distance_m: float) -> PoseStamped:
    """Function that will take the robot's current pose in the map frame and
    determine what the next target point should be according to the original
    pure pursuit paper.
    https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
    

    Specifically, this implementation looks at which point in the path array
    is closest the lookahead distance away from the vehicle sequentially
    after the point that is currently closest to the car.

    Args:
        current_pose (PoseStamped): The robot's current pose in the map
        frame / w.r.t. the map frame.
        path (Path): A sequence of robot poses, each with respect to the map
        frame as well.

    Returns:
        PoseStamped: The point in the path chosen as the next target point.
    """
    # Create a vectorized version of the (x,y) positions in the path using
    # numpy. Also extract the position from the current pose as an ndarray.
    numpy_path = numpy_array_from_path(path=path)
    current_position = numpy_position_from_pose(pose=current_pose.pose.pose)

    # Apply a euclidean distance elementwise across the vector of (x,y) path
    # positions.
    distances = get_distance_to_each_point(current_position=current_position, 
                                           numpy_path=numpy_path)
    # Find the point on the path that is currently closest to the current pose.
    # I.e., the point whose distance is the smallest.

    closest_index = get_smallest_index(distances=distances)
    # Next, we want to find which point AFTER the closest point in our path
    # (sequentially) is the closest to being one lookahead distance away from
    # the car.
    # 
    # To do this, first get a view of the path with only the elements of the
    # path that come after the point closest to the car.
    # distances_after_closest_m = get_distances_after_index(distances=distances, 
    #                                                       index=closest_index)
    # TODO: Shouldn't we actually be including that closest point? What if we
    # have very sparse waypoints and the closest point happens to be the best
    # shot we've got? Unless this might (with sparse waypoints) cause the robot
    # to turn back and steer towards the closest point behind it? In the
    # original paper, they include this point. Maybe the rule should be that the
    # lookahead distance needs to be > than the largest euclidean distance
    # between consecutive points?
    distances_starting_at_closest = get_distances_starting_at_index(distances=distances, 
                                                                    index=closest_index)

    # Next, subtract the lookahead distance from each of these
    # points--"normalizing" them.
    normalized_distances_m = normalize_distances(lookahead_distance_m=lookahead_distance_m,
                                                 distances_m=distances_starting_at_closest)

    # Finally, select the point with the smallest value among them--this is the
    # node whose distance is closest to one lookahead distance away.
    target_point_index = get_smallest_index(distances=normalized_distances_m)

    # Return the pose from the path that corresponds to this target point index.
    # This will be the computed target point index offset by the closest index.
    return path.poses[closest_index+target_point_index]
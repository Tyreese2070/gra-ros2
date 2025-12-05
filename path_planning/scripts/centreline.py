#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Bool
from common_msgs.msg import ConeArray
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from fsd_path_planning import PathPlanner, MissionTypes

"""
TODO:
ROS2 stuff
"""



class CentrelineAlgorithm:
    """Centreline path planning algorithm"""
    # Ask control about look ahead distance

    def __init__(self, lookahead_distance: float = 10.0, max_cone_distance: float = 3.0):
        self.lookahead_distance = lookahead_distance
        self.max_cone_distance = max_cone_distance
    
    def calculate_path(self, yellow_cones, blue_cones, car_position, car_direction):
        """Main entry point - returns waypoint path"""

        # Check for valid cones
        if yellow_cones.shape[0] == 0 or blue_cones.shape[0] == 0:
            return None
        
        # Filter visible cones ahead of the car
        yellow_ahead = self._filter_ahead_of_car(yellow_cones, car_position, car_direction)
        blue_ahead = self._filter_ahead_of_car(blue_cones, car_position, car_direction)

        if yellow_ahead.shape[0] == 0 or blue_ahead.shape[0] == 0:
            return None
        
        pairs = self._pair_cones(yellow_ahead, blue_ahead)
        if len(pairs) == 0:
            return None
        
        midpoints = self._calculate_midpoints(pairs)
        ordered_waypoints = self._order_waypoints(midpoints, car_position)

        # Format Mx3 (index, x, y)
        path = np.zeros((ordered_waypoints.shape[0], 3), dtype=np.float32)
        path[:, 0] = np.arange(ordered_waypoints.shape[0])  # waypoint index
        path[:, 1:3] = ordered_waypoints # x, y positions

        return path

    
    def _filter_ahead_of_car(self, cones, car_position, car_direction):
        """Keep only cones ahead within lookahead distance"""

        filtered_cones = []

        for cone in cones:
            vector_to_cone = cone - car_position
            distance_to_cone = np.linalg.norm(vector_to_cone)

            # Check if within lookahead distance and ahead of the car
            if distance_to_cone <= self.lookahead_distance:
                unit_vector_to_cone = vector_to_cone / distance_to_cone
                dot_product = np.dot(unit_vector_to_cone, car_direction)
                if dot_product > 0:
                    filtered_cones.append(cone)

        return np.array(filtered_cones)
    
    def _pair_cones(self, yellow_cones, blue_cones):
        """Match yellow-blue cone pairs by proximity"""
        pairs = []
        paired_blue_cones = set()

        for cone in yellow_cones:
            distance = np.linalg.norm(blue_cones - cone, axis=1)

            # find nearest blue cone within max_cone_distance
            sorted = np.argsort(distance)

            for blue_index in sorted:
                if blue_index not in paired_blue_cones:
                    if distance[blue_index] <= self.max_cone_distance:
                        pairs.append((cone, blue_cones[blue_index]))
                        paired_blue_cones.add(blue_index)
                    break
        return pairs
    
    def _calculate_midpoints(self, cone_pairs):
        """Calculate midpoint between each pair"""
        midpoints = []
        
        for yellow_cone, blue_cone in cone_pairs:
            midpoint = (yellow_cone + blue_cone) / 2.0
            midpoints.append(midpoint)
        return np.array(midpoints)
    
    def _order_waypoints(self, waypoints, car_position):
        """Sort waypoints nearest to farthest from car"""
        if len(waypoints) == 0:
            return waypoints
        
        # distance from car to each waypoint
        distances = np.linalg.norm(waypoints - car_position, axis=1)

        # sort by distance nearest to farthest
        sorted_indices = np.argsort(distances)

        return waypoints[sorted_indices]


class CentrelineTrackPathfinder(Node):
    """ ROS2 node for centreline path """
    
    def __init__(self):
        self.create_subscription(ConeArray, '/perfect_cone_array', self.cone_callback, 10)
        self.create_subscription(Odometry, '/perfect_odom', self.odom_callback, 10)
        self.get_logger().info("TrackPathfinder node initialized.")

        super().__init__('centreline_track_pathfinder')

        self.centreline_planner = CentrelineAlgorithm(lookahead_distance=10.0, max_cone_distance=3.0)
        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def odom_callback(self, msg):
        """Store vehicle position"""
        self.latest_odom = msg
    
    def cone_callback(self, msg):
        """
            Receive cones, call algorithm, publish path
            NOTE: I need to check if perfect_cone_array and perfect_odom have the same output format as previous topics
        """
        # Organize cones by type (order: unknown, yellow, blue, orange, large_orange)
        # transform to odom frame if necessary
        # get map -> odom
        try:
            tf_map_to_odom = self.tf_buffer.lookup_transform(
                "map",
                "odom",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to odom: {ex}')
            return
        
        if msg.header.frame_id == "map":
            for cone_list in [msg.unknown_cones, msg.yellow_cones, msg.blue_cones, msg.orange_cones, msg.large_orange_cones]:
                for cone in cone_list:
                    point_stamped = PointStamped()
                    point_stamped.header.frame_id = "map"
                    point_stamped.header.stamp = rclpy.time.Time()
                    point_stamped.point = cone.position

                    try:
                        transformed_point = self.tf_buffer.transform(
                            point_stamped,
                            "odom",
                            timeout=rclpy.duration.Duration(seconds=1.0)
                        )
                        cone.position = transformed_point.point
                    except Exception as e:
                        self.get_logger().warn(f"Transform failed for cone: {e}")
        global_cones_temp = [
            np.array([[cone.position.x, cone.position.y] for cone in msg.unknown_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.yellow_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.blue_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.orange_cones], dtype=np.float32).reshape(-1, 2),
            np.array([[cone.position.x, cone.position.y] for cone in msg.large_orange_cones], dtype=np.float32).reshape(-1, 2),
        ]
        global_cones = global_cones_temp  # Preserve fixed order: [unknown, yellow, blue, orange, large_orange]

        if all(c.shape[0] == 0 for c in global_cones):
            self.get_logger().warn("All cone arrays are empty, skipping path planning.")
            return

        # Use odometry if available, else default
        if self.latest_odom is not None:
            car_position = np.array([
                self.latest_odom.pose.pose.position.x,
                self.latest_odom.pose.pose.position.y
            ])
            q = self.latest_odom.pose.pose.orientation
            # Convert quaternion to yaw
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            car_direction = np.array([np.cos(yaw), np.sin(yaw)])
        else:
            car_position = np.array([0.0, 0.0])
            car_direction = np.array([1.0, 0.0])

        # Call path planner
        yellow_cones = global_cones[1]
        blue_cones = global_cones[2]
        try:
            path = self.centreline_planner.calculate_path(yellow_cones, blue_cones, car_position, car_direction)
            if path is None:
                self.get_logger().warn("Planner returned no path.")
                return
            self.publish_path(path)
        except Exception as e:
            self.get_logger().warn(f"Path planning failed: {e}")
    
    def publish_path(self, path):
        """Convert algorithm output to ROS Path message"""
        pass


def main(args=None):
    """ROS2 entry point"""
    pass
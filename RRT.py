import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class RRTPathPlanner(Node):
    def __init__(self):
        super().__init__('rrt_path_planner')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.map_data = None
        self.resolution = None
        self.origin = None      

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def is_collision(self, point):
        """Check for collision with obstacles."""
        x, y = point
        map_x = int((x - self.origin[0]) / self.resolution)
        map_y = int((y - self.origin[1]) / self.resolution)

        # Ensure indices are within bounds
        if map_x < 0 or map_x >= self.map_data.shape[1] or map_y < 0 or map_y >= self.map_data.shape[0]:
            return True  

        return self.map_data[map_y, map_x] > 50  # Occupancy threshold


    def plan_path(self, start, goal, max_iter=500, step_size=0.5):
        """Plan path using RRT."""
        tree = [start]
        for _ in range(max_iter):
            rand_point = self.sample_random_point()
            nearest_point = self.find_nearest(tree, rand_point)
            new_point = self.steer(nearest_point, rand_point, step_size)

            if not self.is_collision(new_point):
                tree.append(new_point)
                if self.is_goal_reached(new_point, goal):
                    return tree + [goal]
        return None

    def sample_random_point(self):
        """Sample a random point in the map."""
        x = np.random.uniform(self.origin[0], self.origin[0] + self.map_data.shape[1] * self.resolution)
        y = np.random.uniform(self.origin[1], self.origin[1] + self.map_data.shape[0] * self.resolution)
        return (x, y)

    def find_nearest(self, tree, point):
        """Find the nearest point in the tree to the given point."""
        return min(tree, key=lambda t: np.linalg.norm(np.array(t) - np.array(point)))

    def steer(self, from_point, to_point, step_size):
        """Move from 'from_point' towards 'to_point' by 'step_size'."""
        direction = np.array(to_point) - np.array(from_point)
        length = np.linalg.norm(direction)
        direction = direction / length if length > 0 else direction
        return tuple(np.array(from_point) + step_size * direction)

    def is_goal_reached(self, point, goal, threshold=0.2):
        """Check if the goal is reached."""
        return np.linalg.norm(np.array(point) - np.array(goal)) < threshold

    def publish_goal(self, goal):
        """Publish the goal pose."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    rrt_planner = RRTPathPlanner()

    start = (0.0, 0.0)  # Starting point (map frame coordinates)
    goal = (2.0, 4.0)   # Goal point (map frame coordinates)

    while rrt_planner.map_data is None:
        rclpy.spin_once(rrt_planner)

    path = rrt_planner.plan_path(start, goal)

    if path:
        for point in path:
            rrt_planner.get_logger().info(f"Point: {point}")
        rrt_planner.publish_goal(goal)
    else:
        rrt_planner.get_logger().error("Path planning failed.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

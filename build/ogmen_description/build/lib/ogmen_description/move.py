# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from math import radians

# class WaypointFollower(Node):
#     def __init__(self):
#         super().__init__('waypoint_follower')
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.timer = self.create_timer(0.1, self.move)

#         # Define waypoints
#         self.waypoints = [[0,0,0], [5,0,45], [5,5,90], [0,5,0], [0,0,135]]
#         self.current_index = 0

#     def move(self):
#         waypoint = self.waypoints[self.current_index]
#         if self.reached_waypoint(waypoint):
#             self.current_index = (self.current_index + 1) % len(self.waypoints)
#         else:
#             self.move_towards_waypoint(waypoint)

#     def reached_waypoint(self, waypoint):
#         # Check if current position is close to the waypoint
#         # Implement your logic here, e.g., using odometry data or sensor data
#         return False

#     def move_towards_waypoint(self, waypoint):
#         # Calculate linear and angular velocities to move towards the waypoint
#         # Implement your logic here
#         # Example: publish Twist message to cmd_vel topic
#         twist = Twist()
#         twist.linear.x = 0.1  # Example linear velocity (m/s)
#         twist.angular.z = radians(5)  # Example angular velocity (radians/s)
#         self.cmd_vel_pub.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     waypoint_follower = WaypointFollower()
#     rclpy.spin(waypoint_follower)
#     waypoint_follower.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import sqrt, atan2, radians

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.waypoints = [[0, 0, 0], [5, 0, 45], [5, 5, 90], [0, 5, 0], [0, 0, 135]]
        self.current_index = 0
        self.waypoint_tolerance = 0.1  # Tolerance for reaching waypoint
        self.align_tolerance = 0.1  # Tolerance for aligning towards waypoint
        self.is_rotating = False
        self.rotation_count = 0
        self.max_rotation_count = 3

    def odom_callback(self, msg):
        if self.is_rotating:
            if self.rotation_count < self.max_rotation_count:
                self.rotate_clockwise()
            else:
                self.is_rotating = False
        else:
            current_pose = msg.pose.pose
            waypoint = self.waypoints[self.current_index]
            distance_to_waypoint = sqrt((waypoint[0] - current_pose.position.x) ** 2 + (waypoint[1] - current_pose.position.y) ** 2)
            if distance_to_waypoint < self.waypoint_tolerance:
                self.stop_robot()
                self.align_towards_waypoint(current_pose, waypoint)
                self.current_index = (self.current_index + 1) % len(self.waypoints)
            else:
                self.move_towards_waypoint(current_pose, waypoint)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def move_towards_waypoint(self, current_pose, waypoint):
        twist = Twist()
        angle_to_waypoint = atan2(waypoint[1] - current_pose.position.y, waypoint[0] - current_pose.position.x)
        twist.linear.x = 0.1  # Example linear velocity (m/s)
        twist.angular.z = self.calculate_angular_velocity(current_pose.orientation.z, angle_to_waypoint)  # Example angular velocity (radians/s)
        self.cmd_vel_pub.publish(twist)

    def align_towards_waypoint(self, current_pose, waypoint):
        angle_to_waypoint = atan2(waypoint[1] - current_pose.position.y, waypoint[0] - current_pose.position.x)
        while abs(current_pose.orientation.z - angle_to_waypoint) > self.align_tolerance:
            twist = Twist()
            twist.angular.z = self.calculate_angular_velocity(current_pose.orientation.z, angle_to_waypoint)  # Example angular velocity (radians/s)
            self.cmd_vel_pub.publish(twist)

    def calculate_angular_velocity(self, current_yaw, target_yaw):
        angular_velocity = 0.5 * (target_yaw - current_yaw)
        return angular_velocity

    def rotate_clockwise(self):
        twist = Twist()
        twist.angular.z = radians(30)  # Example angular velocity (radians/s)
        self.cmd_vel_pub.publish(twist)
        self.rotation_count += 1

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    rclpy.spin(waypoint_follower)
    waypoint_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

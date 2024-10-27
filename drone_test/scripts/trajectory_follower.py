#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import math

class TrajectoryFollower:
    def __init__(self):
        rospy.init_node('trajectory_follower', anonymous=True)
        
        # Subscriber to receive trajectory waypoints
        rospy.Subscriber('/planning_vis/trajectory', Marker, self.marker_callback)
        rospy.Subscriber('/waypoint_generator/waypoints', Path, self.waypoint_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Publisher to send velocity commands
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Parameters for goal tracking
        self.goal_points = []
        self.reach_threshold = 0.1  # Distance threshold to consider a point reached
        self.speed = 0.2  # Set an appropriate linear speed
        self.angular_speed = 1.0  # Set an appropriate angular speed
        
        self.current_goal_index = 0  # Start at the first goal
        self.curr_pose = None
        self.curr_orientation = None
        self.new_trajectory_received = False
        
        # Set the rate for the loop to send velocity commands
        self.rate = rospy.Rate(10)  # 10 Hz

    def marker_callback(self, msg):
        # Update goal points from Marker points (assuming POINTS type Marker with trajectory)
        self.goal_points = [(point.x, point.y) for point in msg.points]
        self.current_goal_index = 0  # Reset to first goal
        self.new_trajectory_received = True  # Indicate that a new trajectory was received

    def waypoint_callback(self, msg):
        # Update goal points from Path waypoints
        self.goal_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.current_goal_index = 0  # Reset to first goal
        self.new_trajectory_received = True  # Indicate that a new trajectory was received

    def odom_callback(self, msg):
        # Update current pose and orientation from odometry
        self.curr_pose = msg.pose.pose.position
        self.curr_orientation = msg.pose.pose.orientation

    def get_distance_and_angle_to_goal(self, goal_x, goal_y):
        # Calculate the Euclidean distance to the goal
        distance = math.sqrt((goal_x - self.curr_pose.x) ** 2 + (goal_y - self.curr_pose.y) ** 2)
        
        # Calculate the angle to the goal
        angle_to_goal = math.atan2(goal_y - self.curr_pose.y, goal_x - self.curr_pose.x)
        
        # Convert quaternion to yaw angle (robot's current orientation in the plane)
        siny_cosp = 2 * (self.curr_orientation.w * self.curr_orientation.z + self.curr_orientation.x * self.curr_orientation.y)
        cosy_cosp = 1 - 2 * (self.curr_orientation.y ** 2 + self.curr_orientation.z ** 2)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate the angular difference
        angle_diff = angle_to_goal - current_yaw
        return distance, angle_diff

    def move_to_goal(self):
        while not rospy.is_shutdown():
            # Check if there's a new trajectory update
            if self.new_trajectory_received:
                rospy.loginfo("New trajectory received. Resetting goals.")
                self.current_goal_index = 0
                self.new_trajectory_received = False
            
            # If all goal points are reached
            if self.current_goal_index >= len(self.goal_points):
                rospy.loginfo("All goal points reached!")
                self.stop()
                rospy.sleep(1)
                continue  # Loop to keep checking for new trajectories

            # Get the current goal point
            goal_x, goal_y = self.goal_points[self.current_goal_index]
            
            # Calculate distance and angle to the current goal
            distance, angle_diff = self.get_distance_and_angle_to_goal(goal_x, goal_y)
            
            # Check if goal is reached
            if distance < self.reach_threshold:
                rospy.loginfo("Reached goal point %d", self.current_goal_index + 1)
                self.current_goal_index += 1  # Move to the next goal
                continue

            # Create Twist message to send velocity commands
            cmd_vel = Twist()
            cmd_vel.linear.x = self.speed * min(distance, 1.0)  # Limit max linear speed
            cmd_vel.angular.z = self.angular_speed * angle_diff  # Proportional control for turning
            
            # Publish the command
            self.publisher.publish(cmd_vel)
            self.rate.sleep()

    def stop(self):
        # Stop the robot by publishing zero velocities
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)

if __name__ == '__main__':
    try:
        follower = TrajectoryFollower()
        follower.move_to_goal()
    except rospy.ROSInterruptException:
        pass

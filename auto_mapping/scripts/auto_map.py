#!/usr/bin/env python3

import rospy
import json
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, BatteryState
from nav_msgs.msg import Odometry
import os

class TurtleBot3GMapping:
    def __init__(self):
        rospy.init_node("turtlebot3_gmapping", anonymous=True)

        # Publishers & Subscribers
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.battery_sub = rospy.Subscriber("/battery_state", BatteryState, self.battery_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Movement commands
        self.vel_msg = Twist()

        # Battery and Position Tracking
        self.battery_percentage = 0.0
        self.last_position = {"x": 0.0, "y": 0.0, "z": 0.0}

        # Obstacle Avoidance Thresholds
        self.obstacle_distance = 0.6  # Minimum distance before avoiding
        self.safe_distance = 1.0  # Distance considered "clear" for moving forward
        self.rotation_attempts = 0  # Counter for rotation
        self.moving_forward = True  # Track movement state

        rospy.loginfo("TurtleBot3 GMapping Node Started")

    def battery_callback(self, data):
        """Reads battery percentage"""
        self.battery_percentage = data.percentage * 100
        rospy.loginfo(f"Battery Percentage: {self.battery_percentage:.2f}%")

    def odom_callback(self, data):
        """Saves last known position"""
        pos = data.pose.pose.position
        self.last_position = {"x": pos.x, "y": pos.y, "z": pos.z}

    def save_last_position(self):
        """Writes last position to a file"""
        file_path = os.path.expanduser("~/turtlebot_last_position.json")
        with open(file_path, "w") as f:
            json.dump(self.last_position, f, indent=4)
        rospy.loginfo(f"Saved last position: {self.last_position}")

    def laser_callback(self, scan_data):
        """Analyzes LiDAR data and decides movement"""
        ranges = np.array(scan_data.ranges)

        # Ignore 0.00m readings (invalid LiDAR values)
    	ranges[ranges == 0] = np.nan  # Treat 0.00m as NaN (not a number)
    	
    	# Compute valid mean values to prevent incorrect obstacle detection
    	front_distance = np.nanmean(ranges[:30].tolist() + ranges[-30:].tolist()) # Average front readings
    	left_distance = np.nanmean(ranges[60:100])  # Left side
    	right_distance = np.nanmean(ranges[-100:-60])  # Right side
    	
    	rospy.loginfo(f"Front: {front_distance:.2f}m, Left: {left_distance:.2f}m, Right: {right_distance:.2f}m")
    	
    	# If all distances are NaN, assume safe distance
    	if np.isnan(front_distance):
            front_distance = self.safe_distance
            
        # Move forward if there is enough space
    	if front_distance > self.obstacle_distance:
            self.move_forward()
   	else:
            rospy.logwarn("Obstacle detected! Avoiding...")
            self.avoid_obstacle(left_distance, right_distance)

    def move_forward(self):
        """Move forward if path is clear"""
        if not self.moving_forward:  # Prevent unnecessary republishing
            rospy.loginfo("Path clear! Moving forward...")
            self.moving_forward = True
            self.vel_msg.linear.x = 0.2
            self.vel_msg.angular.z = 0.0  # Ensure no rotation
            self.vel_pub.publish(self.vel_msg)

    def avoid_obstacle(self):
        """Avoid obstacles by stopping and rotating"""
        self.moving_forward = False  # Prevent forward movement
        self.vel_msg.linear.x = 0.0  # Stop movement
        self.vel_msg.angular.z = 0.5  # Rotate left
        self.vel_pub.publish(self.vel_msg)
        rospy.sleep(1.0)  # Rotate for 1 second

    def shutdown_handler(self):
        """Stops the robot and saves its position"""
        rospy.loginfo("Shutting down, saving last position...")
        self.save_last_position()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.vel_pub.publish(self.vel_msg)

    def run(self):
        """Main loop"""
        rospy.on_shutdown(self.shutdown_handler)
        rospy.spin()

if __name__ == "__main__":
    try:
        bot = TurtleBot3GMapping()
        bot.run()
    except rospy.ROSInterruptException:
        pass



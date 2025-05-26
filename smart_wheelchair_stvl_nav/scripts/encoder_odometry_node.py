#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf2_ros
import tf_conversions

class EncoderOdometry:
    def __init__(self):
        rospy.init_node('encoder_odometry')
        
        # Hardware parameters
        self.left_pin = rospy.get_param('~left_encoder_pin', 18)
        self.right_pin = rospy.get_param('~right_encoder_pin', 19)
        self.ppr = rospy.get_param('~pulses_per_revolution', 1024)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.15)
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.6)
        
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Encoder counters
        self.left_count = 0
        self.right_count = 0
        
        # Odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Setup encoder interrupts
        GPIO.add_event_detect(self.left_pin, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.right_pin, GPIO.RISING, callback=self.right_encoder_callback)
        
        self.last_time = rospy.Time.now()
        
    def left_encoder_callback(self, channel):
        self.left_count += 1
        
    def right_encoder_callback(self, channel):
        self.right_count += 1
        
    def calculate_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        
        # Calculate distances
        left_distance = (self.left_count * 2 * math.pi * self.wheel_radius) / self.ppr
        right_distance = (self.right_count * 2 * math.pi * self.wheel_radius) / self.ppr
        
        # Reset counters
        self.left_count = 0
        self.right_count = 0
        
        # Calculate velocities and pose
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        self.x += distance * math.cos(self.theta + delta_theta/2.0)
        self.y += distance * math.sin(self.theta + delta_theta/2.0)
        self.theta += delta_theta
        
        # Publish odometry
        self.publish_odometry(current_time, distance/dt, delta_theta/dt)
        self.last_time = current_time
        
    def publish_odometry(self, current_time, linear_vel, angular_vel):
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position = Point(self.x, self.y, 0.0)
        odom_quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        
        # Set velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        encoder_odom = EncoderOdometry()
        rate = rospy.Rate(50)  # 50Hz
        while not rospy.is_shutdown():
            encoder_odom.calculate_odometry()
            rate.sleep()
    except rospy.ROSInterruptException:
        GPIO.cleanup()

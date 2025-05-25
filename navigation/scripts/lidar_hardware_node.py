#!/usr/bin/env python3

import rospy
import serial
import struct
import math
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point32
import sensor_msgs.point_cloud2 as pc2

class LidarHardwareNode:
    def __init__(self):
        rospy.init_node('lidar_hardware_node')
        
        # Hardware parameters
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB1')
        self.baud_rate = rospy.get_param('~baud_rate', 230400)
        self.frame_id = rospy.get_param('~frame_id', 'laser_link')
        
        # Publishers
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.pointcloud_pub = rospy.Publisher('/velodyne_points', PointCloud2, queue_size=10)
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
            rospy.loginfo(f"LiDAR connected on {self.serial_port}")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to connect to LiDAR: {e}")
            return
            
        # Start data acquisition
        self.start_scanning()
        
    def start_scanning(self):
        """Send start command to LiDAR hardware"""
        start_cmd = b'\xA5\x60\x05\x00\x00\x00\x01\x85'  # Example command
        self.serial_conn.write(start_cmd)
        rospy.loginfo("LiDAR scanning started")
        
    def read_lidar_data(self):
        """Read and parse LiDAR data from hardware"""
        while not rospy.is_shutdown():
            try:
                # Read packet header
                header = self.serial_conn.read(8)
                if len(header) < 8:
                    continue
                    
                # Parse data packet
                scan_data = self.parse_scan_packet()
                if scan_data:
                    self.publish_scan(scan_data)
                    self.publish_pointcloud(scan_data)
                    
            except Exception as e:
                rospy.logerr(f"LiDAR data read error: {e}")
                
    def parse_scan_packet(self):
        """Parse raw LiDAR packet into range and angle data"""
        # Implementation depends on your specific LiDAR hardware
        ranges = []
        angles = []
        
        # Read distance measurements
        for i in range(360):  # Assuming 360-degree scan
            data = self.serial_conn.read(2)
            if len(data) == 2:
                distance = struct.unpack('<H', data)[0] / 1000.0  # Convert to meters
                angle = math.radians(i)
                ranges.append(distance)
                angles.append(angle)
                
        return {'ranges': ranges, 'angles': angles}
        
    def publish_scan(self, scan_data):
        """Publish LaserScan message"""
        scan_msg = LaserScan()
        scan_msg.header.stamp = rospy.Time.now()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = 2 * math.pi / len(scan_data['ranges'])
        scan_msg.range_min = 0.1
        scan_msg.range_max = 30.0
        scan_msg.ranges = scan_data['ranges']
        
        self.scan_pub.publish(scan_msg)

if __name__ == '__main__':
    try:
        lidar_node = LidarHardwareNode()
        lidar_node.read_lidar_data()
    except rospy.ROSInterruptException:
        pass

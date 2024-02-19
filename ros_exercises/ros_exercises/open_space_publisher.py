import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import OpenSpace
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

import numpy as np

class Laser_Scan_subber(Node):
    def __init__(self):
        super().__init__('open_space_publisher')
        
        self.declare_parameter('Publish topic', 'open_space')
        self.declare_parameter('Subscriber topic', 'fake_scan')
        
        sub_topic_name = self.get_parameter('Subscriber topic').get_parameter_value().string_value
        pub_topic_name = self.get_parameter('Publish topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            LaserScan,
            sub_topic_name,
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(OpenSpace, pub_topic_name, 10)       
        timer_period = .05  # seconds (needs to be 20Hz)
        
        
    def listener_callback(self, msg):
        self.get_logger().info('I got info')
        
        array = np.array(msg.ranges)
        max_distance = np.max(array)
        max_distance_angle = np.argmax(array)
        
        msg = OpenSpace()
        msg.angle = float(max_distance_angle)
        msg.distance = float(max_distance)
        
        
        self.publisher_.publish(msg)
        self.get_logger().info('data sent')
        
        
def main():
    rclpy.init()
    open_space_publisher = Laser_Scan_subber()
    rclpy.spin(open_space_publisher)
    
    open_space_publisher.destroy_node()
    rclpy.shutdown()
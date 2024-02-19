import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType

import numpy as np

class Laser_Scan(Node):
    def __init__(self):
        super().__init__('fake_scan_publisher')
        
        self.declare_parameter('Publish topic','fake_scan')
        self.declare_parameter('Publish rate', .05)
        self.declare_parameter('Angle min', -2/3*np.pi)
        self.declare_parameter('Angle max', 2/3*np.pi)
        self.declare_parameter('Range min', 1.0)
        self.declare_parameter('Range max', 10.0)
        self.declare_parameter('Angle_increment', np.pi/300)
        
        publish_topic = self.get_parameter('Publish topic').get_parameter_value().string_value
        self.get_logger().info(f'topic2publish2: {publish_topic}')
        
        self.publisher_ = self.create_publisher(LaserScan, publish_topic, 10)
        timer_period = self.get_parameter('Publish rate').get_parameter_value().double_value  # seconds (needs to be 20Hz)
        self.clock = rclpy.time.Time()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

        
    def timer_callback(self):
        scan = LaserScan()
        scan_header = Header()
        scan_header.stamp = self.clock.to_msg()
        scan_header.frame_id = 'base_link'

        scan.header = scan_header
        scan.angle_min = self.get_parameter('Angle min').get_parameter_value().double_value
        scan.angle_max = self.get_parameter('Angle max').get_parameter_value().double_value
        scan.angle_increment = self.get_parameter('Angle_increment').get_parameter_value().double_value
        scan.scan_time =  self.get_parameter('Publish rate').get_parameter_value().double_value
        scan.range_min = self.get_parameter('Range min').get_parameter_value().double_value
        scan.range_max = self.get_parameter('Range max').get_parameter_value().double_value
        
        num_points = np.array([(scan.angle_max - scan.angle_min) / (scan.angle_increment) + 1])
        array = 10*np.random.rand(num_points.astype(int)[0])
        scan.ranges = array.tolist()
        
        publish_topic = self.get_parameter('Publish topic').get_parameter_value().string_value
        self.publisher_ = self.create_publisher(LaserScan, publish_topic, 10)
        
        self.publisher_.publish(scan)
        self.get_logger().info('Fake scan published')

        
        
        
def main(args=None):
    rclpy.init(args=args)
    fake_scanner = Laser_Scan()
    rclpy.spin(fake_scanner)
    
    fake_scanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
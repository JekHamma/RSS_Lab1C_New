import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import tf2_ros
import geometry_msgs
import tf_transformations
import time
from geometry_msgs.msg import TransformStamped
from typing import Any


class dynamic_tf_cam_pub(Node):
    def __init__(self):
        super().__init__('dynamic_tf_cam_publisher')
         # instantiate buffer that the listener will write to
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # broadcaster that will publish the transform
        self.br = tf2_ros.TransformBroadcaster(self)

        timer_period = 1.0 / 20  # hz
        self.timer = self.create_timer(timer_period, self.node_callback)

        self.t = time.perf_counter()
        self.left_x = -.05
        self.right_x = .05
        self.rotation_rate = 1  # Hz
        self.rotation_radius = 2.0  # meters
        
        
    def tf_to_se3(self, transform: TransformStamped.transform) -> np.ndarray:
        """
        Convert a TransformStamped message to a 4x4 SE3 matrix 
        """
        q = transform.rotation
        q = [q.x, q.y, q.z, q.w]
        t = transform.translation
        mat = tf_transformations.quaternion_matrix(q)
        mat[0, 3] = t.x
        mat[1, 3] = t.y
        mat[2, 3] = t.z
        return mat

    def se3_to_tf(self, mat: np.ndarray, time: Any, parent: str, child: str) -> TransformStamped:
        """
        Convert a 4x4 SE3 matrix to a TransformStamped message
        """
        obj = geometry_msgs.msg.TransformStamped()

        # current time
        obj.header.stamp = time.to_msg()

        # frame names
        obj.header.frame_id = parent
        obj.child_frame_id = child

        # translation component
        obj.transform.translation.x = mat[0, 3]
        obj.transform.translation.y = mat[1, 3]
        obj.transform.translation.z = mat[2, 3]

        # rotation (quaternion)
        q = tf_transformations.quaternion_from_matrix(mat)
        obj.transform.rotation.x = q[0]
        obj.transform.rotation.y = q[1]
        obj.transform.rotation.z = q[2]
        obj.transform.rotation.w = q[3]

        return obj
    
            
    def node_callback(self):
        # get the transform from the robot to world.. gets transformStamped object
        try:
            tf_world_to_robot: TransformStamped = self.tfBuffer.lookup_transform('world', 'base_link_gt',
                                                                                 rclpy.time.Time())
        except tf2_ros.TransformException:
            self.get_logger().info('no transform from world to base_link_gt found')
            return

        robot_to_world: np.ndarray = self.tf_to_se3(tf_world_to_robot.transform)

        new_t = time.perf_counter()
        time_elapsed = new_t - self.t

        angle = 2 * np.pi * self.rotation_rate * time_elapsed

# Moon Example
        # Z forward, X right, Y down
        # moon_to_robot_translation = [self.rotation_radius * np.cos(angle), 0, self.rotation_radius * np.sin(angle)]
        # moon_to_robot_translation = np.array(moon_to_robot_translation).T
        # moon_to_robot = np.eye(4)
        # moon_to_robot[:3, -1] = moon_to_robot_translation[:3]

        now = self.get_clock().now()

        # # First way: chain with robot_to_world to produce moon_to_world
        # moon_to_world: np.ndarray = robot_to_world @ moon_to_robot
        # tf_moon_to_world = self.se3_to_tf(moon_to_world, now, parent='world', child='moon')
        # self.br.sendTransform([tf_world_to_robot, tf_moon_to_world])

# Left Camera:
        left_cam_to_robot_translation = [self.left_x, 0, 0]
        left_cam_to_robot_translation = np.array(left_cam_to_robot_translation).T
        left_cam_to_robot = np.eye(4)
        left_cam_to_robot[:3, -1] = left_cam_to_robot_translation[:3]
# Right Camera:
        right_cam_to_left_translation = [2*self.right_x, 0, 0]
        right_cam_to_left_translation = np.array(right_cam_to_left_translation).T
        right_cam_to_left = np.eye(4)
        right_cam_to_left[:3, -1] = right_cam_to_left_translation[:3]
        
        now = self.get_clock().now()
        
 # First way: chain with robot_to_world to produce moon_to_world
        # moon_to_world: np.ndarray = robot_to_world @ moon_to_robot
        # tf_moon_to_world = self.se3_to_tf(moon_to_world, now, parent='world', child='moon')
        # self.br.sendTransform([tf_world_to_robot, tf_moon_to_world])

# First way left camera:
        left_cam_to_world: np.ndarray = robot_to_world @ left_cam_to_robot
        tf_left_cam_to_world = self.se3_to_tf(left_cam_to_world, now, parent='world', child='left_cam')
        self.br.sendTransform([tf_world_to_robot, tf_left_cam_to_world])
        
# First way right camera:
        right_cam_to_left: np.ndarray = right_cam_to_left
        tf_right_cam_to_left = self.se3_to_tf(right_cam_to_left, now, parent='left_cam', child='right_cam')
        self.br.sendTransform([tf_left_cam_to_world, tf_right_cam_to_left])        
        
# Easier way: no need to chain with robot_to_world

        # tf_moon_to_robot = self.se3_to_tf(moon_to_robot, now, parent='base_link_gt', child='moon')
        # self.br.sendTransform([tf_world_to_robot, tf_moon_to_robot])

        self.get_logger().info('Published')



def main(args=None):
    rclpy.init(args=args)
    dynamic_tf_node = dynamic_tf_cam_pub()
    rclpy.spin(dynamic_tf_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dynamic_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
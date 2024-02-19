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


class static_tf_cam_pub(Node):
    def __init__(self):
        super().__init__('static_tf_cam_publisher')
         # instantiate buffer that the listener will write to
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # broadcaster that will publish the transform
        self.br = tf2_ros.StaticTransformBroadcaster(self)

        self.t = time.perf_counter()
        self.left_x = -.05
        self.right_x = .05
        
        self.node_callback()        
        
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
# Moon Example
        # Z forward, X right, Y down
        # moon_to_robot_translation = [self.rotation_radius * np.cos(angle), 0, self.rotation_radius * np.sin(angle)]
        # moon_to_robot_translation = np.array(moon_to_robot_translation).T
        # moon_to_robot = np.eye(4)
        # moon_to_robot[:3, -1] = moon_to_robot_translation[:3]



        # # First way: chain with robot_to_world to produce moon_to_world
        # moon_to_world: np.ndarray = robot_to_world @ moon_to_robot
        # tf_moon_to_world = self.se3_to_tf(moon_to_world, now, parent='world', child='moon')
        # self.br.sendTransform([tf_world_to_robot, tf_moon_to_world])

# Easy Way Left Camera:
        left_cam_to_robot_translation = [self.left_x, 0, 0]
        left_cam_to_robot_translation = np.array(left_cam_to_robot_translation).T
        left_cam_to_robot = np.eye(4)
        left_cam_to_robot[:3, -1] = left_cam_to_robot_translation[:3]
        
        now = self.get_clock().now()
        tf_left_cam_to_robot = self.se3_to_tf(left_cam_to_robot, now, parent='base_link_gt', child='left_cam')


# Easy Way Right Camera:
        right_cam_to_robot_translation = [self.right_x, 0, 0]
        right_cam_to_robot_translation = np.array(right_cam_to_robot_translation).T
        right_cam_to_robot = np.eye(4)
        right_cam_to_robot[:3, -1] = right_cam_to_robot_translation[:3]

        tf_right_cam_to_robot = self.se3_to_tf(right_cam_to_robot, now, parent='base_link_gt', child='right_cam')

# Sending all transforms
        self.br.sendTransform([tf_left_cam_to_robot, tf_right_cam_to_robot])
        # self.br.sendTransform([tf_world_to_robot, tf_right_cam_to_robot])
        self.get_logger().info('Published')




def main(args=None):
    rclpy.init(args=args)
    static_tf_node = static_tf_cam_pub()
    rclpy.spin(static_tf_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    static_tf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from geometry_msgs.msg import Quaternion

from innobot_msgs.action import Pnp

class PickNPlaceActionClient(Node):

    def __init__(self):
        super().__init__('pick_n_place_client')
        self._action_client = ActionClient(self, Pnp, 'calibration_as')
        self.declare_parameters(
            namespace="",
            parameters=[
                ('pick_pose.position.x',None),
                ('pick_pose.position.y',None),
                ('pick_pose.position.z',None),
                ('pick_pose.orientation.roll',None),
                ('pick_pose.orientation.pitch',None),
                ('pick_pose.orientation.yaw',None),
                ('place_pose.position.x',None),
                ('place_pose.position.y',None),
                ('place_pose.position.z',None),
                ('place_pose.orientation.roll',None),
                ('place_pose.orientation.pitch',None),
                ('place_pose.orientation.yaw',None),
                ('object_width',None),            
            ]
        )

    def send_goal(self, secs):
        goal_msg = Pnp.Goal()

        pick_x = self.get_parameter("pick_pose.position.x").get_parameter_value().double_value
        pick_y = self.get_parameter("pick_pose.position.y").get_parameter_value().double_value
        pick_z = self.get_parameter("pick_pose.position.z").get_parameter_value().double_value
        pick_r = self.get_parameter("pick_pose.orientation.roll").get_parameter_value().double_value
        pick_p = self.get_parameter("pick_pose.orientation.pitch").get_parameter_value().double_value
        pick_yw = self.get_parameter("pick_pose.orientation.yaw").get_parameter_value().double_value

        place_x = self.get_parameter("place_pose.position.x").get_parameter_value().double_value
        place_y = self.get_parameter("place_pose.position.y").get_parameter_value().double_value
        place_z = self.get_parameter("place_pose.position.z").get_parameter_value().double_value
        place_r = self.get_parameter("place_pose.orientation.roll").get_parameter_value().double_value
        place_p = self.get_parameter("place_pose.orientation.pitch").get_parameter_value().double_value
        place_yw = self.get_parameter("place_pose.orientation.yaw").get_parameter_value().double_value

        object_width = self.get_parameter("object_width").get_parameter_value().double_value
        
        goal_msg.pick_pose.position.x = pick_x
        goal_msg.pick_pose.position.y = pick_y
        goal_msg.pick_pose.position.z = pick_z
        goal_msg.pick_pose.orientation = self.get_quaternion_from_euler(pick_r, pick_p, pick_yw)

        goal_msg.place_pose.position.x = place_x
        goal_msg.place_pose.position.y = place_y
        goal_msg.place_pose.position.z = place_z
        goal_msg.place_pose.orientation = self.get_quaternion_from_euler(place_r, place_p, place_yw)

        goal_msg.object_width = object_width

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info('Result: {0}'.format(result.status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback))

    def get_quaternion_from_euler(self,roll,pitch,yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        quat = Quaternion(x=qx,y=qy,z=qz,w=qw)
 
        return quat


def main(args=None):
    rclpy.init(args=args)

    action_client = PickNPlaceActionClient()

    future = action_client.send_goal(8)
    executor = MultiThreadedExecutor()

    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()
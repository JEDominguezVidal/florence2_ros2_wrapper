#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image

from florence2_interfaces.action import ExecuteTask

class Florence2ActionExample(Node):
    def __init__(self):
        super().__init__('florence2_action_example')
        
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.image_topic = self.get_parameter('image_topic').value
        
        # Subscribe to the image stream
        self.raw_image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Action Client
        self.action_client = ActionClient(self, ExecuteTask, '/florence2_node/execute_task_action')
        
        self.frame_count = 0
        self.target_frame = 10
        self.action_in_progress = False
        
        self.get_logger().info(f"Subscribed to {self.image_topic}. Waiting for {self.target_frame} frames to trigger action...")

    def image_callback(self, msg):
        if self.action_in_progress:
            return
            
        self.frame_count += 1
        
        if self.frame_count % 2 == 0 and self.frame_count < self.target_frame:
            self.get_logger().info(f"Received {self.frame_count}/{self.target_frame} frames...")
            
        if self.frame_count >= self.target_frame:
            self.get_logger().info("Target frame count reached. Sending asynchronous goal to Florence-2 action server...")
            self.action_in_progress = True
            self.send_goal(msg)

    def send_goal(self, image_msg):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()

        goal_msg = ExecuteTask.Goal()
        goal_msg.task = '<MORE_DETAILED_CAPTION>'
        goal_msg.image = image_msg

        self.get_logger().info(f"Sending goal with task: {goal_msg.task}")
        
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        status = feedback_msg.feedback.status
        self.get_logger().info(f"Action Feedback: {status}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.shutdown_node()
            return

        self.get_logger().info('Goal accepted :) Waiting for result...')
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        self.get_logger().info('Action call successful!')
        self.get_logger().info(f"Task executed: {result.task_prompt}")
        self.get_logger().info(f"Output: {result.text_output}")
        self.get_logger().info('Example completed successfully. Shutting down node.')
        
        self.shutdown_node()

    def shutdown_node(self):
        # We need to explicitly signal rclpy to stop spinning
        import threading
        def _stop():
            rclpy.try_shutdown()
        threading.Thread(target=_stop).start()

def main(args=None):
    rclpy.init(args=args)
    
    node = Florence2ActionExample()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

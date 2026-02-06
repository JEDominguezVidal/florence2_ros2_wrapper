#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from florence2_interfaces.srv import ExecuteTask
import sys
import threading

class Florence2ServiceExample(Node):
    def __init__(self):
        super().__init__('florence2_service_example')
        
        self.declare_parameter('image_topic', '/camera/image_raw')
        image_topic = self.get_parameter('image_topic').value
        
        # We will count incoming frames.
        # Note: 10 frames is an arbitrary number chosen for this example to demonstrate
        # triggering a service call based on a condition. You can change this logic
        # in your own application (e.g., trigger when a robot reaches a waypoint).
        self.frame_count = 0
        self.target_frames = 10
        self.service_called = False
        self.lock = threading.Lock()
        
        # Subscribe to the image topic
        self.image_sub = self.create_subscription(
            ImageMsg,
            image_topic,
            self.image_callback,
            10
        )
        
        # Publisher for the annotated result
        self.annotated_pub = self.create_publisher(ImageMsg, '~/example_annotated_image', 10)
        
        # Client for the Florence-2 ExecuteTask service
        self.florence2_client = self.create_client(ExecuteTask, '/florence2_node/execute_task')
        
        self.get_logger().info(f"Subscribed to {image_topic}. Waiting for {self.target_frames} frames to trigger service...")

    def image_callback(self, msg):
        with self.lock:
            if self.service_called:
                return  # We only want to execute this process once
            
            self.frame_count += 1
            if self.frame_count % 2 == 0:
                self.get_logger().info(f"Received {self.frame_count}/{self.target_frames} frames...")
                
            if self.frame_count >= self.target_frames:
                self.service_called = True
                self.get_logger().info("Target frame count reached. Calling Florence-2 service <OD>...")
                self.call_florence2_service(msg)

    def call_florence2_service(self, image_msg):
        # Wait for service to be available
        while not self.florence2_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Florence-2 service not available, waiting again...')
            
        req = ExecuteTask.Request()
        req.task = '<OD>'
        req.image = image_msg # Send the specific image we want to analise
        
        future = self.florence2_client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service call successful!")
            self.get_logger().info(f"Task executed: {response.task_prompt}")
            self.get_logger().info(f"Output: {response.text_output}")
            
            # Note: The main node /florence2_node automatically publishes the annotated image 
            # to its ~/annotated_image topic if it parses the task correctly.
            # In a real scenario, you can read response.parsed_json and draw your own boxes,
            # but for simplicity, we will just log that the node should be shutting down now.
            
            self.get_logger().info("Example completed successfully. Shutting down node.")
            
        except Exception as e:
            self.get_logger().error(f"Service call failed %r" % (e,))
        finally:
            # Shutdown after one execution
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = Florence2ServiceExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass # Handle rclpy.shutdown() graceful exit
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()

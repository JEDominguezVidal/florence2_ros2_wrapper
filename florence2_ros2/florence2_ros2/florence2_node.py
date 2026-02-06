import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import json
import torch
from transformers import AutoProcessor, AutoModelForCausalLM
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import threading

from florence2_interfaces.srv import ExecuteTask

class Florence2Node(Node):
    def __init__(self):
        super().__init__('florence2_node')
        
        # Declare parameters
        self.declare_parameter('model_name', 'microsoft/Florence-2-large-ft')
        self.declare_parameter('continuous_task', '')
        self.declare_parameter('image_topic', '/camera/image_raw')
        
        model_name = self.get_parameter('model_name').value
        self.continuous_task = self.get_parameter('continuous_task').value
        image_topic = self.get_parameter('image_topic').value
        
        # Initialise device and model
        self.get_logger().info(f"Initialising Florence-2 model: {model_name}")
        
        # Force CUDA initialization to fix CUBLAS_STATUS_NOT_INITIALIZED
        if torch.cuda.is_available():
            torch.cuda.init()
            # Create a dummy tensor to fully initialize cuBLAS
            _ = torch.zeros(1).cuda()
            
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32
        
        self.model = AutoModelForCausalLM.from_pretrained(
            model_name,
            torch_dtype=self.torch_dtype,
            trust_remote_code=True
        ).to(self.device)
        self.processor = AutoProcessor.from_pretrained(model_name, trust_remote_code=True)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_lock = threading.Lock()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            ImageMsg,
            image_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.annotated_pub = self.create_publisher(ImageMsg, '~/annotated_image', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, '~/detections', 10)
        self.results_pub = self.create_publisher(String, '~/results_json', 10)
        
        # Services
        self.execute_task_srv = self.create_service(
            ExecuteTask,
            '~/execute_task',
            self.execute_task_callback
        )
        
        self.get_logger().info("Florence-2 node is ready.")
        
    def image_callback(self, msg):
        with self.latest_image_lock:
            self.latest_image = msg
            
        # Process in continuous mode
        if self.continuous_task:
            self.process_task(self.continuous_task, "", msg)

    def execute_task_callback(self, request, response):
        self.get_logger().info(f"Received request for task: {request.task}")
        # Use provided image if not empty, otherwise latest image
        target_image_msg = request.image if hasattr(request.image, 'data') and request.image.data else self.latest_image
        
        if target_image_msg is None:
            self.get_logger().error("No image available for processing.")
            response.task_prompt = request.task
            response.text_output = "Error: No image available."
            response.parsed_json = "{}"
            return response
            
        text_output, parsed_answer = self.process_task(request.task, request.text_input, target_image_msg)
        
        response.task_prompt = request.task
        response.text_output = text_output
        response.parsed_json = json.dumps(parsed_answer)
        
        return response
        
    def process_task(self, task, text_input, image_msg):
        # Convert ROS image to PIL Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            pil_image = Image.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return "", {}
            
        prompt = task
        if text_input:
            prompt += text_input
            
        # Run inference
        inputs = self.processor(text=prompt, images=pil_image, return_tensors="pt").to(self.device, self.torch_dtype)
        
        generated_ids = self.model.generate(
            input_ids=inputs["input_ids"],
            pixel_values=inputs["pixel_values"],
            max_new_tokens=1024,
            num_beams=3
        )
        
        generated_text = self.processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
        parsed_answer = self.processor.post_process_generation(generated_text, task=task, image_size=(pil_image.width, pil_image.height))
        
        self.publish_results(task, parsed_answer, pil_image, image_msg.header)
        
        return generated_text, parsed_answer

    def publish_results(self, task, parsed_answer, pil_image, header):
        # Publish generic JSON
        json_msg = String()
        json_msg.data = json.dumps(parsed_answer)
        self.results_pub.publish(json_msg)
        
        # Handle object detection explicitly to publish Detection2DArray and Annotated Image
        if task in parsed_answer:
            task_data = parsed_answer[task]
            if isinstance(task_data, dict) and "bboxes" in task_data and "labels" in task_data:
                bboxes = task_data["bboxes"]
                labels = task_data["labels"]
                
                # Publish Detection2DArray
                det_array = Detection2DArray()
                det_array.header = header
                for bbox, label in zip(bboxes, labels):
                    det = Detection2D()
                    det.header = header
                    # bbox format is [x1, y1, x2, y2]
                    x1, y1, x2, y2 = bbox
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    w = float(x2 - x1)
                    h = float(y2 - y1)
                    
                    det.bbox.center.position.x = cx
                    det.bbox.center.position.y = cy
                    det.bbox.size_x = w
                    det.bbox.size_y = h
                    
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(label)
                    hyp.hypothesis.score = 1.0 # Model doesn't output confidence scores by default
                    det.results.append(hyp)
                    
                    det_array.detections.append(det)
                    
                self.detections_pub.publish(det_array)
                
                # Annotate image
                annotated_img = self.draw_bboxes(pil_image.copy(), bboxes, labels)
                annotated_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(np.array(annotated_img), cv2.COLOR_RGB2BGR), encoding="bgr8")
                annotated_msg.header = header
                self.annotated_pub.publish(annotated_msg)

    def draw_bboxes(self, image, bboxes, labels):
        draw = ImageDraw.Draw(image)
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
        except:
            font = ImageFont.load_default()
            
        colors = ["#FF0000", "#00FF00", "#0000FF", "#FFFF00", "#FF00FF", "#00FFFF", "#FFA500", "#800080"]
        
        for i, (bbox, label) in enumerate(zip(bboxes, labels)):
            color = colors[i % len(colors)]
            x1, y1, x2, y2 = bbox
            
            draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
            # Use textbbox for text background if available
            try:
                text_bbox = draw.textbbox((x1, y1), str(label), font=font)
                draw.rectangle([text_bbox[0]-2, text_bbox[1]-2, text_bbox[2]+2, text_bbox[3]+2], fill=color)
            except AttributeError:
                pass
            draw.text((x1, y1), str(label), fill="white", font=font)
            
        return image

def main(args=None):
    rclpy.init(args=args)
    node = Florence2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import cv2
import base64
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from psyche_interfaces.action import InferenceWithImages
from .inference_client import InferenceClient

class ContinuousVision(InferenceClient):
    def __init__(self):
        super().__init__('continuous_vision', "inspect", InferenceWithImages)
        self.image_queue = []
        self.image_subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        self.sensation_publisher = self.create_publisher(String, 'sensation', 10)
        self.get_logger().info("Continuous Vision node started")
        self.busy = False
        self.timer = self.create_timer(1, self.handle_queue)
    def on_sentence(self, sentence: str):
        self.get_logger().info(f"Received sentence: {sentence}")
        self.sensation_publisher.publish(String(data=sentence))    
    def on_result(self, result: str):
        self.get_logger().info(f"Received result: {result}")  
    def handle_queue(self):
        if self.busy:
            self.get_logger().info("Busy, deferring to queue")
            return
        
        if len(self.image_queue) < 2:  # Need at least two frames to make a difference
            self.get_logger().info("Not enough frames in queue")
            return
        
        self.busy = True
        
        # Calculate differences between consecutive frames
        difference_images = []
        for i in range(len(self.image_queue) - 1):
            current_frame = self.decode_image(self.image_queue[i])
            next_frame = self.decode_image(self.image_queue[i + 1])
            diff = cv2.absdiff(current_frame, next_frame)
            difference_images.append(diff)
        
        # Create a composite image from the differences
        composite_image = self.make_action_over_time_image(difference_images)
        self.image_queue = []  # Clear the queue after processing
        self.busy = False
        
        self.handle_queue() # Immediately start processing again
        
    def decode_image(self, encoded):
        ''' Decode base64 encoded image to an OpenCV image '''
        img_data = base64.b64decode(encoded)
        np_arr = np.fromstring(img_data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return img
        
    def make_action_over_time_image(self, images):
        ''' Make a single image from multiple difference images '''
        height = min(img.shape[0] for img in images)
        width = sum(img.shape[1] for img in images)
        composite_image = np.zeros((height, width, 3), dtype=np.uint8)
        
        x_offset = 0
        for img in images:
            composite_image[:, x_offset:x_offset + img.shape[1]] = img
            x_offset += img.shape[1]
        
        return composite_image
    
    def handle_queue(self):
        if self.busy:
            self.get_logger().info(f"Busy, deferring to queue")
            return
        self.get_logger().info(f"handling image queue")
        if len(self.image_queue) < 1:
            self.get_logger().info(f"Queue is empty")
            return
        self.busy = True        
        self.get_logger().info(f"Handling queue of {len(self.image_queue)} images")
        head = self.image_queue[0]
        tail = self.image_queue[-1]
        middle = self.make_action_over_time_image(self.image_queue[1:-1])
        images = [head, middle, tail]
        counter = 1
        for image in images:
            filename = f"image{counter}.jpg"
            with open(filename, "wb") as file:
                file.write(base64.b64decode(image))
            counter += 1
        self.image_queue = []
        self.get_logger().info(f"Queue cleared")
        self.infer("The images show the beginning, middle and end of a moment. What do the images contain? Speak only of the content of the images, not the images themselves.", {'images': images})
        self.get_logger().info(f"Triggered inference. Queue handled.")
            
    def generate_prompt(self, prompt_template: str, inputs: dict = {}):
        self.get_logger().info(f"Generating prompt")
        images = inputs.get('images', [])
        self.get_logger().info(f"Found {len(images)} images")
        shrunken_inputs = {k: v for k, v in inputs.items() if k != 'images'}
        self.get_logger().info(f"Other inputs: {shrunken_inputs}")
        response = super().generate_prompt(prompt_template, shrunken_inputs)
        self.get_logger().info(f"Got the base prompt {response}")
        return { "prompt": response["prompt"], "images": images }
    def image_callback(self, msg):
        processed_image = self.process_frame(msg)
        self.image_queue.append(processed_image)

    def process_frame(self, image):
        np_arr = np.frombuffer(image.data, np.uint8)
        img_yuv = np_arr.reshape((image.height, image.width, 2))
        img_bgr = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR_YUY2)
        _, img = cv2.imencode('.jpg', img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 30])
        encoded = str(base64.b64encode(img).decode('utf-8'))
        return encoded

def main(args=None):
    rclpy.init(args=args)
    continuous_vision = ContinuousVision()
    rclpy.spin(continuous_vision)
    continuous_vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

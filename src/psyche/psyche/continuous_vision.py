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
        self.counter = 0
        self.image_queue = []
        self.image_subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        self.sensation_publisher = self.create_publisher(String, 'sensation', 90)
        self.get_logger().info("Continuous Vision node started")
        self.busy = False
        self.timer = self.create_timer(1, self.handle_queue)
    
    def on_sentence(self, sentence: str):
        self.get_logger().info(f"Received sentence: {sentence}")
        self.sensation_publisher.publish(String(data=sentence))    
    
    def on_result(self, result: str):
        self.get_logger().info(f"Received result: {result}")  
        self.busy = False
        
    def decode_image(self, encoded_image):
        ''' Decode a base64 encoded image to a NumPy array suitable for OpenCV processing '''
        data = base64.b64decode(encoded_image)
        np_arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return img
    
    def save_image(self, image, image_type):
        ''' Save the image to disk '''
        _, encoded = cv2.imencode('.jpg', image)
        base64_encoded = base64.b64encode(encoded).decode('utf-8')
        filename = f"{image_type}_image{self.counter}.jpg"
        with open(filename, "wb") as file:
            file.write(base64.b64decode(base64_encoded))
        self.counter += 1
        self.get_logger().info(f"{image_type.capitalize()} image saved as {filename}")

    def handle_queue(self):
        if self.busy:
            self.get_logger().info("Busy, deferring to queue")
            return
        
        if len(self.image_queue) < 2:
            self.get_logger().info("Not enough frames in queue")
            return
        
        self.busy = True
        
        # Decode images and calculate differences between consecutive frames
        difference_images = []
        for i in range(len(self.image_queue) - 1):
            current_frame = self.decode_image(self.image_queue[i])
            next_frame = self.decode_image(self.image_queue[i + 1])
            diff = cv2.absdiff(current_frame, next_frame)
            difference_images.append(diff)
        
        composite_image = self.make_action_over_time_image(difference_images)
        _, comp_encoded = cv2.imencode('.jpg', composite_image)
        comp_base64 = base64.b64encode(comp_encoded).decode('utf-8')
            
        images = [self.image_queue[0], self.image_queue[-1], comp_base64]
              
        self.infer(f"""These are footage captured by a robot over a span of time. The first image is the first frame, the second is the last frame and the third is a composite of the differences between frames in the intervening space—a kind of motion blur of the intervening space. Describe what is happening in the images. Refer to the content of the images and not the images themselves: instead of saying "The image shows a man," say "There is a man".""", {"images": images})
        
        self.image_queue = []
        self.get_logger().info("Queue cleared")

    def make_action_over_time_image(self, images):
        ''' Overlay images by averaging them '''
        # Convert all images to float type for averaging
        # avg_image = np.zeros(images[0].shape, np.float32)
        avg_image = images[0].astype(np.float32)
        for img in images:
            avg_image += img.astype(np.float32)
        
        # Divide by the number of images to average
        avg_image /= len(images)
        
        avg_image += 128  # Add 128 to make the image brighter
                
        # Convert back to 8-bit to save as JPEG
        avg_image = np.clip(avg_image, 0, 255).astype(np.uint8)
        
        return avg_image
            
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
        _, img = cv2.imencode('.png', img_bgr)#, [cv2.IMWRITE_JPEG_QUALITY, 30])
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

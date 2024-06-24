import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import base64
import numpy as np

class ContinuousVision(Node):
    def __init__(self):
        super().__init__('continuous_vision')
        self.image_queue = []
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10
        )

        def image_callback(self, msg):
            processed_image = self.process_frame(msg)
            self.image_queue.append(processed_image)

        def process_frame(self, image):
            # Open the image
            img = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)

            # Scale the image to 670x670px

            img = cv2.resize(img, (670, 670))

            # Compress the image as PNG
            _, img_encoded = cv2.imencode('.png', img)

            # Convert the image to base64-encoded string
            img_base64 = base64.b64encode(img_encoded).decode('utf-8')

            return img_base64
        
def main(args=None):
    rclpy.init(args=args)
    continuous_vision = ContinuousVision()
    rclpy.spin(continuous_vision)
    continuous_vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
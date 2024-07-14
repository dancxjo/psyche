import rclpy
from rclpy.node import Node
import cv2
import base64
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from psyche_interfaces.action import PlainTextInference
from .inference_client import InferenceClient

class SensationIntegration(InferenceClient):
    def __init__(self):
        super().__init__('sensation_integration', "instruct")
        self.sensation_queue = []
        self.sensation_sub = self.create_subscription(
            String,
            'sensation',
            self.sensation_callback,
            50
        )
        self.situation_publisher = self.create_publisher(String, 'situation', 1)
        self.voice_publisher = self.create_publisher(String, 'voice', 1)
        self.current_situation = ""
        self.busy = False
        self.get_logger().info("Sensation integration node started")
    
    def on_sentence(self, sentence: str):
        self.get_logger().info(f"Received sentence: {sentence}")
        # self.voice_publisher.publish(String(data=sentence))
    
    def on_result(self, result: str):
        self.get_logger().info(f"Received result: {result}")
        self.current_situation = result
        self.situation_publisher.publish(String(data=self.current_situation))
        self.busy = False
        self.infer_situation()
            
    def sensation_callback(self, msg):
        self.sensation_queue.append(msg.data)
        if not self.busy:
            self.infer_situation()
            
    def infer_situation(self):
        if self.busy:
            self.get_logger().debug("Busy, deferring to queue")
            return
        
        if len(self.sensation_queue) < 1:
            self.get_logger().info("Not enough sensations in queue")
            return
        
        self.busy = True
        self.infer("""You are an robot control system embedded in physical hardware. These are your most recent sensations: {sensations}\nLast you knew, this was your situation: {situation}\nUsing this information, describe your situation. Based only on the information from the sensations listed above, maintain a detailed transcript of the conversation you're having (if you're having one) and what's going on around you. Only transcribe what you witness. Do not try to act here. Simply maintain a log of what's happening. You will hear your own voice; keep track of it. (When you hear something like 'Robot 1 initialized,' you're up and running.)""", {"sensations": str(self.sensation_queue), "situation": self.current_situation})
        self.sensation_queue = []

def main(args=None):
    rclpy.init(args=args)
    continuous_vision = SensationIntegration()
    rclpy.spin(continuous_vision)
    continuous_vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

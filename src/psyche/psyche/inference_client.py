import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from psyche_interfaces.action import PlainTextInference, InferenceWithImages

import numpy as np

class InferenceClient(Node):
    """
    A node that listens to one or more topics, transforms them via an LPU and publishes the topic
    """
    def __init__(self, node_name, action_server_name: str = 'instruct', inference_type = PlainTextInference):
        super().__init__(node_name)
        self.Inference = inference_type
        self.action_client = ActionClient(self, self.Inference, action_server_name)        

    def generate_prompt(self, prompt_template: str, inputs: dict = {}):
        '''A convenient place to override to make custom clients, such as for llava with image inputs'''
        return {"prompt" :prompt_template.format(**inputs)}

    def is_valid_input(self, prompt_template: str, inputs: dict) -> bool:
        return True

    def infer(self, prompt_template: str, inputs: dict = {}):
        if not self.is_valid_input(prompt_template, inputs):
            self.get_logger().error('Invalid input')
            # TODO: We probably shouldn't swallow this error
            return
        
        input = self.generate_prompt(prompt_template, inputs)
        self.get_logger().debug(f"Generated prompt: {input}")
        self.get_logger().debug(f'Prompt: {input}; awaiting action server')
        goal = self.Inference.Goal(**input)
        self.action_client.wait_for_server()
        self.get_logger().info(f"Action server found")
        future = self.action_client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_inferred)
        
    def on_chunk(self, chunk: str):
        '''A hook for raw chunks sent back from the inference server'''
        pass
    
    def on_word(self, word: str):
        '''A hook for units of text _on par_ with "words". In the last sentence ["_", "on", "par_", "with", '"', "words"""] is more like it.'''
        pass
    
    # This is helpful if you are sending this to a TTS node
    def on_sentence(self, sentence: str):
        '''A hook for full sentences. This is the most useful for most applications, like streaming to /voice. (/voice will get its own sentence splitting, but this allows asynchronous speech and yields to other nodes in the meantime.)'''
        pass
        
    def on_result(self, result: str):
        '''A hook for the final result. Use the on_sentence hook if you can possibly use the results in a streaming fashion. This is the last hook called.'''
        pass
        
    def goal_response_callback(self, future) -> None:
        """
        Callback for handling the response after sending the goal.
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().debug('Goal rejected :(')
                return

            self.get_logger().debug('Goal accepted :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.on_response)
        except Exception as e:
            self.get_logger().error(f'Error in goal response callback: {e}')

    def on_inferred(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return
            
            self.get_logger().debug('Goal accepted')
            future = goal_handle.get_result_async()
            future.add_done_callback(self.on_response)
        except Exception as e:
            self.get_logger().error(f'Error in inference done: {e}')
    
    def on_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        
        self.get_logger().debug(f'Feedback: {feedback.chunk}')
        
        if feedback.granularity == 0:
            self.on_chunk(feedback.chunk)
        elif feedback.granularity == 1:
            self.on_word(feedback.chunk)
        elif feedback.granularity == 2:
            self.on_sentence(feedback.chunk)
        else:
            self.get_logger().error('Unknown feedback level')
    
    def on_response(self, future):
        try:
            result = future.result().result
            self.on_result(result.response)
        except Exception as e:
            self.get_logger().error(f'Error in processing response: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    node = InferenceClient('inference_client')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
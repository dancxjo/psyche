import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from .language_processor import LanguageProcessor
from psyche_interfaces.action import InferenceWithImages
import requests
import json
import base64
from sentence_splitter import split_text_into_sentences
import re

class VisionEnabledLanguageProcessor(LanguageProcessor):
    def __init__(self, node_name, action_server_name="instruct"):
        """
        Initializes the node, the language model, and the action server.
        """
        # TODO: Make a higher abstraction for LPUs that's abstract, with a new LangChainLP that does all the "default" stuff
        super().__init__(node_name)
        action_server_name = self.get_parameter("action_server_name").get_parameter_value().string_value
        self.action_server_name = action_server_name
        self._action_server = ActionServer(
            self,
            InferenceWithImages,
            action_server_name,
            self.receive_goal_callback
        )
        
    def extract_input(self, goal_handle):
        """
        Extracts the input from the goal handle.
        """
        input_data = goal_handle.request
        return {
            "prompt": input_data.prompt,
            "images": input_data.images
        }
        
        
    def stream(self, goal_handle):
        """
        Processes the request in chunks and provides feedback.
        """
        result = InferenceWithImages.Result()
        result.response = ""
        sentence_buffer = ""
        word_buffer = ""
        input_data = self.extract_input(goal_handle)
        payload = {
            "model": self.model,
            "prompt": input_data["prompt"],
            "images": input_data["images"]
        }
        headers = {'Content-Type': 'application/json'}

        try:
            self.get_logger().debug(f"Sending request")
            with requests.post(f"{self.base_url}/api/generate", json=payload, headers=headers, stream=True) as response:
                self.get_logger().debug(f"Sent request")
                response.raise_for_status()
                self.get_logger().debug(f"Received good response")
                for line in response.iter_lines():
                    self.get_logger().debug(f"Got line")
                    line = line.strip()
                    if line:
                        self.get_logger().debug(f"Decoding {line}")
                        decoded_line = json.loads(line.decode('utf-8'))
                        self.get_logger().debug(f"Unmarshalled as: {decoded_line}")
                        chunk = decoded_line.get("response", "")
                        self.get_logger().debug(f"Extracted {chunk}")
                        if not chunk:
                            continue
                        result.response += chunk
                        self.get_logger().debug(f"Concatted {result.response}")
                        self.report_chunk(goal_handle, chunk, 0)
                        self.get_logger().debug(f"Chunk fed back")
                        
                        # Update and process word and sentence buffers
                        word_buffer, sentence_buffer = self.buffer_chunks(goal_handle, chunk, word_buffer, sentence_buffer)
                        self.get_logger().debug(f"Buffers: {word_buffer}, {sentence_buffer}")
                        
                        
                        if decoded_line.get('done', False):
                            self.get_logger().debug(f"Completed")
                            break

                        self.get_logger().debug(f"Looping")
                        
            # Final flush for any remaining text in buffers
            if word_buffer:
                self.report_chunk(goal_handle, word_buffer, 1)
            if sentence_buffer:
                self.report_chunk(goal_handle, sentence_buffer.strip(), 2)

            goal_handle.succeed()
            return result

        except requests.RequestException as e:
            self.get_logger().error(f'API call failed: {str(e)}')
            goal_handle.abort()
            return result

        except Exception as e:
            self.get_logger().error(f'Exception during goal execution: {str(e)}')
            goal_handle.abort()
            return result
        
def main(args=None):
    rclpy.init(args=args)

    node = VisionEnabledLanguageProcessor('vision_enabled_language_processor')

    rclpy.spin(node)


if __name__ == '__main__':
    main()

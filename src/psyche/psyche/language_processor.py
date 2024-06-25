import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from psyche_interfaces.action import PlainTextInference, InferenceWithImages

from langchain_community.llms import Ollama
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate
from langchain_core.output_parsers import StrOutputParser

from sentence_splitter import split_text_into_sentences
import re

class LanguageProcessor(Node):
    """
    An action server that exposes a langchain.
    """
    def __init__(self, node_name, action_server_name="instruct"):
        """
        Initializes the node, the language model, and the action server.
        """
        super().__init__(node_name)
        self.declare_parameter("action_server_name", action_server_name)
        self.declare_parameter("image_support", False)
        action_server_name = self.get_parameter("action_server_name").get_parameter_value().string_value
        self.action_server_name = action_server_name
        self.supports_images = self.get_parameter("image_support").get_parameter_value().bool_value
        self.Inference = InferenceWithImages if self.supports_images else PlainTextInference
        self.prompt = PromptTemplate.from_template("{input}")
        self.initialize_langchain()
        self._action_server = ActionServer(
            self,
            self.Inference,
            action_server_name,
            self.receive_goal_callback
        )

    def setup_llm(self):
        self.declare_parameter('model', 'llama3:instruct')
        self.declare_parameter('base_url', 'http://127.0.0.1:11434')
        self.declare_parameter('model_type', 'ollama')

        self.model = self.get_parameter('model').get_parameter_value().string_value
        self.base_url = self.get_parameter('base_url').get_parameter_value().string_value
        self.model_type = self.get_parameter('model_type').get_parameter_value().string_value
        
        if self.model_type == 'openai':
            self.llm = ChatOpenAI(model=self.model)
        else:
            self.llm = Ollama(model=self.model, base_url=self.base_url, temperature=0.9, num_predict=256)
        
        self.get_logger().debug('Language model initialized with model: {0}'.format(self.model))

    def setup_output_parser(self):
        self.output_parser = StrOutputParser()
        self.get_logger().debug('Output parser initialized.')

    def setup_chain(self):
        self.chain = self.prompt | self.llm | self.output_parser
        self.get_logger().debug('Processing chain created.')

    def initialize_langchain(self):
        """
        Sets up the language model and related components based on ROS parameters.
        """
        self.setup_llm()
        self.setup_output_parser()
        self.setup_chain()
        
    def receive_goal_callback(self, goal_handle):
        """
        Receives and processes a goal using the language model chain.
        """
        self.get_logger().debug(f'Streaming to {self.action_server_name}...')
        result = self.stream(goal_handle)
        return result

    def extract_input(self, goal_handle):
        """
        Extracts the input from the goal handle.
        """
        return {"input": goal_handle.request.prompt}

    def on_sentence(self, sentence):
        self.get_logger().info(f"Received sentence: {sentence}")

    def report_chunk(self, goal_handle, chunk, chunk_level=0):
        self.get_logger().debug(f"Reporting chunk")
        feedback_msg = self.Inference.Feedback()
        feedback_msg.chunk = chunk
        feedback_msg.granularity = chunk_level
        self.get_logger().debug(f"{feedback_msg}")
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().debug(f"fed back")

    def buffer_chunks(self, goal_handle, chunk, word_buffer, sentence_buffer):
        word_buffer += chunk
        self.get_logger().debug(f'Word buffer: {word_buffer}')

        words = [word for word in re.split(r"(?<!\w'\w)\b", word_buffer) if word != ""]
        self.get_logger().debug(f'Words: {words}')
        is_word_complete = len(words) > 1
        if is_word_complete:
            first_word = words[0]
            if first_word.strip() != "":
                self.report_chunk(goal_handle, first_word, 1)
            index = word_buffer.index(first_word)
            end = index + len(first_word)
            word_buffer = word_buffer[end:]
        
        sentence_buffer += chunk
        self.get_logger().debug(f'Sentence buffer: {sentence_buffer}')                 
        sentences = split_text_into_sentences(sentence_buffer, language="en")
        self.get_logger().debug(f'Sentences: {sentences}')
        is_sentence_complete = len(sentences) > 1
        if is_sentence_complete:
            first_sentence = sentences[0]
            if first_sentence.strip() != "":
                self.report_chunk(goal_handle, first_sentence.strip(), 2)
            index = sentence_buffer.index(first_sentence)
            end = index + len(first_sentence)
            sentence_buffer = sentence_buffer[end:]
        return word_buffer, sentence_buffer

    def stream(self, goal_handle):
        """
        Processes the request in chunks and provides feedback.
        """
        result = self.Inference.Result()
        result.response = ""
        sentence_buffer = ""
        word_buffer = ""
        try:
            for chunk in self.chain.stream(self.extract_input(goal_handle)):
                if chunk:
                    chunk = str(chunk)
                    result.response += chunk
                    self.report_chunk(goal_handle, chunk, 0)

                    word_buffer, sentence_buffer = self.buffer_chunks(goal_handle, chunk, word_buffer, sentence_buffer)

                    if goal_handle.is_cancel_requested:
                        self.get_logger().debug('Goal canceled by client.')
                        goal_handle.canceled()
                        return result
                    
            # Flush the buffers
            if word_buffer:
                self.report_chunk(goal_handle, word_buffer.strip(), 1)
            if sentence_buffer:
                self.report_chunk(goal_handle, sentence_buffer.strip(), 2)
            goal_handle.succeed()
            return result
        except Exception as e:
            self.get_logger().error('Exception during goal execution: {0}'.format(str(e)))
            goal_handle.abort()
            return result

def main(args=None):
    rclpy.init(args=args)

    node = LanguageProcessor('language_processor')

    rclpy.spin(node)


if __name__ == '__main__':
    main()
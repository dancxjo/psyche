from .informant import Informant
from .language_processor import LanguageProcessor
from langchain_community.document_loaders import TextLoader, DirectoryLoader
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings, ChatOpenAI
from langchain_text_splitters import CharacterTextSplitter, Language, RecursiveCharacterTextSplitter
from langchain_community.embeddings import OllamaEmbeddings
from langchain.storage import LocalFileStore
from langchain.embeddings import CacheBackedEmbeddings
from langchain_core.runnables import RunnablePassthrough
from langchain_core.prompts import PromptTemplate, ChatPromptTemplate, MessagesPlaceholder
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from create_msgs.msg import DefineSong, PlaySong
from langchain.agents import AgentExecutor, tool, create_json_chat_agent
from langchain.agents.format_scratchpad.openai_tools import format_to_openai_tool_messages
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain_community.llms import Ollama
from langchain import hub

class Recall(Informant):
    """
    An informant that uses the RAG system to answer questions.
    """
    def setup_documents(self):
        self.declare_parameter('memory_path', '/psyche/memory/data/pages')
        memory_path = self.get_parameter('memory_path').get_parameter_value().string_value
        self.declare_parameter('memory_glob', '**/*.txt')
        memory_glob = self.get_parameter('memory_glob').get_parameter_value().string_value
        self.loader = DirectoryLoader(memory_path, glob=memory_glob)
        self.raw_docs = self.loader.load()
        self.text_splitter = RecursiveCharacterTextSplitter.split_documents()
        self.split_docs = self.text_splitter.split_documents(self.raw_docs)
        self.documents = self.split_docs
        self.get_logger().info('Documents loaded and split.')
        # TODO: How do we invalidate this? How do we add new documents?


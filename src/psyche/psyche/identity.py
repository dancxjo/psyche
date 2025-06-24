from .distiller import Distiller
from .informant import Informant
from .language_processor import LanguageProcessor
from langchain_community.document_loaders import TextLoader, DirectoryLoader
from langchain_community.vectorstores import Qdrant
from qdrant_client import QdrantClient
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
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String

from langchain_core.prompts import PromptTemplate

from psyche_interfaces.action import PlainTextInference

import yaml
import re
import json
import requests

# Global variables for DokuWiki API access
API_URL = "http://127.0.0.1:9000/lib/exe/jsonrpc.php"
HEADERS = {'Content-Type': 'application/json'}

prompt = """Use all the information available to you to answer the question: "Who is PETE?" Please phrase it in the first person as you are him. So...who are you? Extra points for economy of words. Use only a few sentences."""

class Identity(Distiller):
    def __init__(self):
        super().__init__('identity')
        self.prompt = prompt

    def on_result(self, result: str):
        self.output_pub.publish(String(data=result))

def main(args=None):
    rclpy.init(args=args)
    node = Identity()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

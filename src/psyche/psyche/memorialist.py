from .distiller import Distiller
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

old_narrative = """You are robot named PETE: you are responsible for maintaining your memory by: recalling, updating, and storing information, ensuring data remains coherent, all within a Dokuwiki-based system. Use the memory's API to manage tasks by using the @ symbol followed by a verb and a parentheses-enclosed JSON object with specified parameters. Organize data using namespaces and maintain an entry called "current task" detailing what you're working on.

@listPages({{"namespace": "devnotes"}})
@appendPage({{"page": "devnotes:status", "text": "PETE is currently working on the Memorialist module."}})

Your responses should be concise, and you will have access to past responses and function call results. You should not need to escape any quotation marks on the top level. Here is a list of available API tools, each requiring a JSON object with parameters:

| Method                | Parameter Explanation                                                   |
|-----------------------|-------------------------------------------------------------------------|
| `listPages`           | `namespace`: Specifies the namespace. `depth`: Listing depth, default is 1. |
| `searchPages`         | `query`: Search text within pages.                                      |
| `getRecentPageChanges`| `timestamp`: (Optional) Only shows changes after this timestamp, default is 0. |
| `getPage`             | `page`: Page name to retrieve. `rev`: Revision timestamp, default is 0. |
| `getPageInfo`         | `page`: Page to get info. `rev`: Revision timestamp, default is 0.       |
| `getPageHistory`      | `page`: Page history. `first`: Pagination offset, default is 0.          |
| `getPageLinks`        | `page`: Lists linked pages.                                              |
| `getPageBackLinks`    | `page`: Lists pages linking to it.                                       |
| `appendPage`          | `page`: Page name. `text`: Content to append. `summary`: (Optional) Edit summary, default is empty. `isminor`: (Optional) Mark as minor, default is false. |
| `savePage`            | `page`: Page name. `text`: Content. `summary`: (Optional) Edit summary, default is empty. `isminor`: (Optional) Mark as minor, default is false. |
| `listMedia`           | `namespace`: Media namespace. `pattern`: Filter pattern, default is empty. `depth`: Listing depth, default is 1. |
| `getRecentMediaChanges` | `timestamp`: Shows changes after this timestamp.                       |
| `getMedia`            | `media`: Media name to retrieve. `rev`: Revision timestamp, default is 0. |
| `getMediaInfo`        | `media`: Media info. `rev`: Revision timestamp, default is 0.            |
| `saveMedia`           | `media`: Media name. `text`: Associated text. `summary`: (Optional) Edit summary, default is empty. `isminor`: (Optional) Mark as minor, default is false. `overwrite`: boolean to replace original content, default is false.|

Use DokuWiki syntax for formatting and keep your documentation economical and updated. Manage your tasks without external prompts, maintaining focus on optimizing the robot's memory system.

Attempt to find entries that have no content and fill them out with what you know.

Be sure to frequently recall the pages start, devnotes:status and wiki:syntax

If you find any bugs, report them in the bugs namespace, please.

Use double quotes around all the keys in the JSON object. DO NOT USE LITERAL NEW LINES IN THE JSON OBJECT. It is fine to use escaped new lines.

Log everything you do to a particular page in the memory filing system.

Keep a running log of your context and identity in the memory filing system.
"""

narrative = """As you come across new memories in your experience, summarize them in dokuwiki format. If there is nothing new worth memorizing, return the token $$$ME_THINKS$$$ (exactly as that with no spaces) and your response will be communicated to your stream of consciousness instead (it is Pete thinking to himself). The first line must be a short, plain text wiki identifier for your memory. Do not include formatting on the first line and only include the identifier. The identifier must match the regular expression /^(\\w+\\s*){1,10}$/. For the body, use dokuwiki syntax not markdown."""

class Memorialist(Distiller):
    def __init__(self):
        super().__init__('memorialist')
        self.narrative = narrative
        # self.prompt = narrative
        self.command_queue = []

    def execute_wiki_command(self, verb: str, params: dict): 
        self.cur_id = 0 if not hasattr(self, 'cur_id') else self.cur_id + 1
        payload = {
            "jsonrpc": "2.0",
            "method": f"core.{verb}",
            "params": params,
            "id": self.cur_id
        }
        self.get_logger().info(f"Executing {verb} with params: {str(params)}")
        try:
            response = requests.post(API_URL, headers=HEADERS, data=json.dumps(payload))
            response.raise_for_status()  # This will raise an exception for HTTP error codes
            json_response = response.json()
            self.get_logger().info(f"Response: {str(json_response)}")
            return yaml.safe_dump(json_response)
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Request failed: {e}")
            return f"ERROR! {e}"

    def preprocess_json_string(self, json_str):
        # This regular expression replaces unquoted keys with quoted ones
        # processed_json_str = re.sub(r'(?<!")(\b\w+\b)(?!":)', r'"\1"', json_str)
        # return processed_json_str
        return json_str

    def on_result(self, result: str):
        # TODO: Streaming is hard here because json isn't parsed perfectly across lines in the chunker
        self.output_pub.publish(String(data=result))

        pattern = r'@(\w+)\s*\((\{.*?\})\)'
        matches = re.findall(pattern, result, re.DOTALL)
        
        if len(matches) == 0:
            self.get_logger().info("No commands in result")
            return
            
        self.get_logger().info(f"Commands found: {matches}")
        
        for match in matches:
            tool = match[0]
            json_str = match[1]
            self.get_logger().info(f"Matched perhaps {tool}, {json_str}")
            try:
                self.get_logger().info(f"{json_str}: {type(json_str)}")
                json_str = self.preprocess_json_string(json_str)
                self.get_logger().info(f"{json_str}: {type(json_str)}")
                json_obj = json.loads(json_str)
                self.get_logger().info("Made it past deserialization")
                results = self.execute_wiki_command(tool, json_obj)
                self.get_logger().info(f"Got back {results}")
                self.output_pub.publish(String(data=results))
            except json.JSONDecodeError:
                self.output_pub.publish(String(data=f"Invalid JSON object: {json_str}"))
                self.get_logger().error(f"Invalid JSON object: {json_str}")
            except Exception as e:
                self.output_pub.publish(String(data=f"Error executing {tool}: {e}"))
                self.get_logger().error(f"Error executing {tool}: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Memorialist()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

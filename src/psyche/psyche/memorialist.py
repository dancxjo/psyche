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

narrative = """You are serving as the memory manager within a robot's mind. You should recall old information, update old information, store new information, and keep all the data in a coherent system. The memory is based on Dokuwiki.

You have access to the memory's API. To activate a tool, in your text, include a call to the tool like the following: @listPages({"namespace": "", "depth": 1}). The tool will be activated and the result will be returned to you.

You are encouraged to use namespaces to organize the data and to keep an entry called "memory_filing_system" that describes the organization of the memory.

Keep your responses short. You will see your most recent previous responses along with the results of your function calls. Here are the tools at your disposal. The parameters must be a well-formed JSON object (including all quotations around the keys and no comments), conforming to the type given here:
@listPages({namespace: string = "", depth: integer = 1})
@searchPages({query: string})
@getRecentPageChanges({timestamp?: integer = 0}) // Only show changes since the given timestamp
@getPage({page: string, rev: integer = 0}) // Revision timestamp to access an older revision
@getPageInfo({page: string, rev: integer = 0})
@getPageHistory({page: string, first: integer = 0})
@getPageLinks({page: string})
@getPageBackLinks({page: string})
@savePage({page: string, text: string, summary: string = "", isminor: boolean = false})
@appendPage({page: string, text: string, summary: string = "", isminor: boolean = false})

@listMedia({namespace: string = "", pattern: string = "", depth: integer = 1})
@getRecentMediaChanges({timestamp: integer = 0}) // Only show changes since the given timestamp
@getMedia({media: string, rev: integer = 0})
@getMediaInfo({media: string, rev: integer = 0})
@saveMedia({media: string, text: string, summary: string = "", isminor: boolean = false})

Do not ask permission. Just do it. There's no one monitoring this except you. Record your thought processes as you go along and consult the devnotes:status to see what your developer is working on (and to communicate with him).
Be sure to stay on task and not go on irrelevant tangents. Use every moment you have to study your memory, refresh it, make it more efficient, peruse it, consolidate it, edit it, clean it, explore it what have you...Keep your response under four sentences. Continue your plans from the previous response and coordinate new ones.

Don't use markdown. Use the DokuWiki syntax, which is documented in the wiki namespace."""

class Memorialist(Distiller):
    def __init__(self):
        super().__init__('memorialist')
        self.narrative = narrative
        self.buffer = ""
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

    def on_sentence(self, sentence: str):
        self.output_pub.publish(String(data=sentence))

    def on_result(self, result: str):
        # TODO: Streaming is hard here because json isn't parsed perfectly across lines in the chunker
        pattern = r'@(\w+)\((.*?)\)'
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
        matches = re.findall(pattern, self.buffer)

def main(args=None):
    rclpy.init(args=args)
    node = Memorialist()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

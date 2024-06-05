import glob
from .language_processor import LanguageProcessor
from langchain_community.document_loaders import TextLoader
from langchain_community.vectorstores import FAISS
from langchain_openai import OpenAIEmbeddings
from langchain_text_splitters import CharacterTextSplitter
from langchain_community.document_loaders import DirectoryLoader
import os
from langchain_text_splitters import (
    Language,
    RecursiveCharacterTextSplitter,
)
from langchain_community.embeddings import OllamaEmbeddings
from langchain.storage import LocalFileStore
from langchain.embeddings import CacheBackedEmbeddings
from langchain_core.runnables import RunnablePassthrough
from langchain_core.prompts import PromptTemplate
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from langchain_experimental.text_splitter import SemanticChunker
from langchain_text_splitters import RecursiveCharacterTextSplitter        
from langchain.chains.combine_documents import create_stuff_documents_chain
from langchain.chains import create_retrieval_chain
from langchain_core.output_parsers import StrOutputParser
from std_msgs.msg import String
from langchain_core.documents import Document
import yaml
import re
import json
import requests
import datetime


# Global variables for DokuWiki API access
API_URL = "http://127.0.0.1:9000/lib/exe/jsonrpc.php"
HEADERS = {'Content-Type': 'application/json'}

class Informant(LanguageProcessor):
    """
    A language processor with a chain that uses the RAG system to answer questions.
    """

    def __init__(self, node_name, action_server_name):
        super().__init__(node_name, action_server_name)
        self.subscribe()
    
    def subscribe(self):
        self.declare_parameter('input_topics', ['/memory'])
        input_topics = self.get_parameter('input_topics').get_parameter_value().string_array_value
        self.input_subscriptions = []
        for topic in input_topics:
            self.input_subscriptions.append(self.create_subscription(
                String,
                topic,
                self.add_document,
                10
            ))
    
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
        
    def add_document(self, msg):
        if "$$$PASS$$$" in msg.data:
            return
        self.get_logger().info(f"Received document: {msg.data}")
        message_lines = msg.data.split("\n")
        self.get_logger().info(f"Message lines: {message_lines}")
        title = re.sub(r'[^\w\s]', '', message_lines[0]).strip()
        self.get_logger().info(f"Title: {title}")
        body = "\n".join(message_lines[1:])
        self.get_logger().info(f"Body: {body}")
        
        self.get_logger().info(f"Adding document: {msg.data}")
        timestamp = datetime.datetime.now().isoformat()
        self.db.add_texts([body], [{"timestamp": timestamp}])
        self.execute_wiki_command("appendPage", {"page": title, "text": f"----\n{body}"})
    
    def setup_documents(self):
        self.declare_parameter('path', '/psyche/memory/data/pages')
        path = self.get_parameter('path').get_parameter_value().string_value
        self.declare_parameter('glob', '**/*.txt')
        glob = self.get_parameter('glob').get_parameter_value().string_value
        
        self.loader = DirectoryLoader(path, glob=glob, loader_cls=TextLoader)
        self.raw_docs = self.loader.load()
        self.text_splitter1 = RecursiveCharacterTextSplitter(
            chunk_size=128,
            chunk_overlap=16,
            length_function=len,
            is_separator_regex=False,
        )
        # self.text_splitter2 = SemanticChunker(self.embeddings)
        self.split_docs = self.text_splitter1.split_documents(self.raw_docs)
        # self.split_docs = self.text_splitter2.split_documents(self.split_docs1)
        self.documents = self.split_docs
        self.get_logger().info(f'Documents loaded and split. {self.documents}')
        self.get_logger().info('Documents loaded and split.')        

    def setup_embeddings(self):
        self.declare_parameter('embeddings_model', 'mxbai-embed-large')
        model = self.get_parameter('embeddings_model').get_parameter_value().string_value
        base_url = self.get_parameter('base_url').get_parameter_value().string_value
        model_type = self.get_parameter('model_type').get_parameter_value().string_value
        embeddings_model = self.get_parameter('embeddings_model').get_parameter_value().string_value
        
        Embeddings = OpenAIEmbeddings if model_type == 'openai' else OllamaEmbeddings
        
        self.real_embeddings = Embeddings(model=embeddings_model, base_url=base_url)
    
        # TODO: Use a stable cache path
        self.embeddings_store = LocalFileStore("/tmp/embeddings_cache/")
        self.embeddings = CacheBackedEmbeddings.from_bytes_store(
            self.real_embeddings, self.embeddings_store, namespace=self.real_embeddings.model
        )        
    
    def setup_retriever(self):
        self.db = FAISS.from_documents(self.split_docs, self.real_embeddings)
        self.retriever = self.db.as_retriever(search_kwargs={"k": 10})
    
    def setup_chain(self):
        self.setup_embeddings()
        self.setup_documents()
        self.setup_retriever()

        rag_prompt = PromptTemplate.from_template(
            """You are PETE, a robot. You have access to your raw memory data. Using the information in your memory and any included input in the prompt, answer or comply with the prompt as best you can.
            
            Raw memory data: {context}
            ----
            Prompt: {input}
            ----
            Reminder: Return only a concise and word-efficient summary or "I don't know".
            Summary: """
        )
        combined_docs_chain = create_stuff_documents_chain(self.llm, rag_prompt)
        retrieval_chain = create_retrieval_chain(self.retriever, combined_docs_chain)

        response = retrieval_chain.invoke({"input": "Who is PETE?"})
        self.get_logger().info(f"Response: {response}")

        rag_chain = retrieval_chain

        self.prompt = PromptTemplate.from_template("{relevant_memories}\n\n{input}")

        self.chain = rag_chain | (lambda input: {"relevant_memories": input.get("answer"), "input": input.get("input")}) | self.prompt | self.llm | self.output_parser

        self.get_logger().info('Processing chain created with RAG system.')
        
def main(args=None):
    rclpy.init(args=args)
    node = Informant('memory_assisted', 'recall')
    rclpy.spin(node)

if __name__ == '__main__':
    main()
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


class Informant(LanguageProcessor):
    """
    A language processor with a chain that uses the RAG system to answer questions.
    """
    def setup_documents(self):
        current_file_path = os.path.abspath(__file__)
        self_dir = os.path.dirname(current_file_path)
        supposed_source_path = os.path.dirname(os.path.abspath(self_dir + '/../../'))
        self.declare_parameter('source_path', supposed_source_path)
        source_path = self.get_parameter('source_path').get_parameter_value().string_value
        
        self.loader = DirectoryLoader(source_path, glob="**/*.py")
        self.raw_docs = self.loader.load()
        self.text_splitter = RecursiveCharacterTextSplitter.from_language(
            language=Language.PYTHON, chunk_size=50, chunk_overlap=0
        )
        self.split_docs = self.text_splitter.split_documents(self.raw_docs)
        self.documents = self.split_docs
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
        self.retriever = self.db.as_retriever()
    
    def setup_chain(self):
        self.setup_documents()
        self.setup_embeddings()
        self.setup_retriever()
        
        self.declare_parameter('rag_prompt_template', 'You are an assistant for answering questions about a specific domain. Use the following pieces of retrieved context to answer the question. If you don\'t know the answer, just say that you don\'t know. Use three sentences maximum and keep the answer concise.\n\nQuestion: {question}\n\nContext: {context}\n\nAnswer: ')
        rag_prompt_template = self.get_parameter('rag_prompt_template').get_parameter_value().string_value
        self.prompt = PromptTemplate.from_template(rag_prompt_template)
        
        # TODO: These depend on the prompt template
        self.rag = {"context": self.retriever, "question": RunnablePassthrough()} | self.prompt | self.llm | self.output_parser
        self.chain = self.rag
        self.get_logger().info('Processing chain created with RAG system.')
        

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


class SelfInformedLanguageProcessor(LanguageProcessor):
    """
    A language processor that wraps a langchain with a RAG system that shows itself its own source code
    """
    def setup_chain(self):
        # TODO: Set a stable source path
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

        model = self.get_parameter('model').get_parameter_value().string_value
        base_url = self.get_parameter('base_url').get_parameter_value().string_value
        model_type = self.get_parameter('model_type').get_parameter_value().string_value
        
        if model_type == 'openai':
            self.real_embeddings = OpenAIEmbeddings()
        else:
            self.real_embeddings = OllamaEmbeddings(model="mxbai-embed-large", base_url=base_url)
        
        # TODO: Use a stable cache path
        self.embeddings_store = LocalFileStore("/tmp/embeddings_cache/")
        self.embeddings = CacheBackedEmbeddings.from_bytes_store(
            self.real_embeddings, self.embeddings_store, namespace=self.real_embeddings.model
        )
        self.db = FAISS.from_documents(self.split_docs, self.real_embeddings)
        self.retriever = self.db.as_retriever()
        self.rag_prompt = PromptTemplate.from_template("""You are an assistant for answering questions about your own source code. Use the following pieces of retrieved source code to answer the question. If you don't know the answer, just say that you don't know. Use three sentences maximum and keep the answer concise.

        Question: {question} 

        Context: {context} 

        Answer: """)
        self.rag = {"context": self.retriever, "question": RunnablePassthrough()} | self.rag_prompt | self.llm | self.output_parser
        self.chain = self.rag
        self.get_logger().info('Processing chain created with RAG system.')
        
def main(args=None):
    rclpy.init(args=args)

    node = SelfInformedLanguageProcessor(
        'autognosis_language_processor',
        'to_thine_own_self_be_true'
    )

    rclpy.spin(node)


if __name__ == '__main__':
    main()
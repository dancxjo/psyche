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
from .wiki_tools import wiki_tools
from langchain_core.prompts import ChatPromptTemplate


@tool
def query_memory(question: str):
    """Ask a question in natural language to engage your retrieval system."""
    pass

class Memorialist(LanguageProcessor):
    def setup_agent(self):
        self.tools = wiki_tools #+ [self.query_memory]
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", """Answer the following questions as best you can. YOU MUST USE THE EXACT FORMAT WITHOUT MAKING EXTRANEOUS COMMENTARY! You have access to the following tools:

            {tools}

            The way you use the tools is by specifying a json blob.
            Specifically, this json should have a `action` key (with the name of the tool to use) and a `action_input` key (with the input to the tool going here).

            The only values that should be in the "action" field are: {tool_names}

            The $JSON_BLOB should only contain a SINGLE action, do NOT return a list of multiple actions. Here is an example of a valid $JSON_BLOB:

            ```
            {{
            "action": $TOOL_NAME,
            "action_input": $INPUT
            }}
            ```

            ALWAYS use the following format:

            Question: the input question you must answer
            Thought: you should always think about what to do
            Action:
            ```
            $JSON_BLOB
            ```
            Observation: the result of the action
            ... (this Thought/Action/Observation can repeat N times)
            Thought: I now know the final answer
            Final Answer: the final answer to the original input question

            Begin! Reminder to always use the exact characters `Final Answer` when responding."""),
            ("human", """{prompt}

            {agent_scratchpad}"""),
            ])
        self.agent = create_json_chat_agent(self.llm, self.tools, self.prompt)
        self.agent_executor = AgentExecutor(agent=self.agent, tools=self.tools, verbose=True, handle_parsing_errors=False)

    def setup_chain(self):
        super().setup_chain()
        self.setup_agent()
        self.chain = self.agent_executor

def main(args=None):
    rclpy.init(args=args)
    node = Memorialist('memorialist', 'manage_memory')
    rclpy.spin(node)

if __name__ == '__main__':
    main()

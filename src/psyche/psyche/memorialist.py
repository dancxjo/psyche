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
from langchain.agents import AgentExecutor, tool, create_json_chat_agent, create_structured_chat_agent
from langchain.agents.format_scratchpad.openai_tools import format_to_openai_tool_messages
from langchain.agents.output_parsers.openai_tools import OpenAIToolsAgentOutputParser
from langchain_community.llms import Ollama
from langchain import hub
from .wiki_tools import wiki_tools
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.messages import SystemMessage, AIMessage, HumanMessage


@tool
def query_memory(question: str):
    """Ask a question in natural language to engage your retrieval system."""
    pass

prompt_template = ChatPromptTemplate.from_messages([
    SystemMessage("""Assistant is serving as the memory manager for a robot's mind. Assistant should recall old information, update old information, store new information and keep all the data in a coherent system. The memory is based on Dokuwiki."""),
    MessagesPlaceholder('chat_history'),
    HumanMessage("""TOOLS
------
Assistant can ask the user to use tools to look up information that may be helpful in answering the users original question. If you're stumped, you might try looking at the start page. The tools the human can use are:

{tools}

RESPONSE FORMAT INSTRUCTIONS
----------------------------

When responding to me, please output a response in one of two formats:

**Option 1:**
Use this if you want the human to use a tool.
Markdown code snippet formatted in the following schema:

```json
{{
    "action": string, \ The action to take. Must be one of {tool_names}
    "action_input": string \ The input to the action
}}
```

**Option #2:**
Use this if you want to respond directly to the human. Markdown code snippet formatted in the following schema:

```json
{{
    "action": "Final Answer",
    "action_input": string \ You should put what you want to return to use here
}}
```

ROBOT'S INPUT
--------------------
Here is the robot's input (remember to respond with a markdown code snippet of a json blob with a single action, and NOTHING else):

{input}"""),
    MessagesPlaceholder('agent_scratchpad'),
])

class Memorialist(LanguageProcessor):
    def setup_agent(self):
        self.tools = wiki_tools #+ [self.query_memory]
        self.prompt = prompt_template
        self.agent = create_json_chat_agent(self.llm, self.tools, self.prompt)
        self.agent_executor = AgentExecutor(agent=self.agent, tools=self.tools, verbose=True, handle_parsing_errors=True)

    def extract_input(self, goal_handle):
        """
        Extracts the input from the goal handle.
        """
        if self.chat_history == None:
            self.chat_history = []
            
        return {"input": goal_handle.request.prompt, "chat_history": self.chat_history},

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

import rclpy
from std_msgs.msg import String
from .inference_client import InferenceClient
import re
from neo4j import GraphDatabase
import chromadb
import ollama
import json

system_message = """In well formatted JSON, list the relevant entities and their relationships as described in the natural language description of the situation. Use the following format:
{ "entities": [{"variable": "me", "category": "Person", "description": "an artificial intelligence named Pete"}, {"variable": "body", "category": "Thing", "description": "a laptop"}], "relationships": [{"source": "body", "target": "pete", "predicate": "EMBODIES"}], "updates": [{ "target": "me", "field": "name", "value": "Pete"}] }. Use only the PascalCased categories of Person, Place, Thing, Event or Idea. Capture other features in the description. As you learn more about nodes, update their fields as shown in the example. Try to capture what's in the situation faithfully. You must define all entities before you can reference them in a relationship."""


old = """You are a graph memory system. You are responsible for storing memories in a graph database. You receive situations and Cypher queries from the user, and you must rewrite the query to correctly translate the situation into a graph. First, identify the entities involved in the situation, create them in the graph database, and establish the relationships between them. As you receive the results of your queries, integrate them into the Cypher translation. You must always use the correct Cypher inside markdown syntax: Any code you write between single backticks or triple backticks will be replace the user's query. Correct the user's query. Use MERGE instead of MATCH to create new entities when needed. Make sure the query matches the situation provided. Also, help find duplicate nodes as you come across them. Include in your script a relationship :IS_ALSO between nodes that are duplicates. Always make sure to return the IDs and values of all the relevant nodes in your query a RETURN ID(a), a, ID(b), b, ID(c), c. Only include one return statement at the end of the command. Only one command at a time. You will see this prompt again.\n\nNext query including database updates and corrections:\n\n"""


class GraphMemory(InferenceClient):
    def __init__(self):
        super().__init__('graph_memory', "code")
        self.situation_sub = self.create_subscription(
            String,
            'situation',
            self.situation_callback,
            1
        )
        self.memories = []
        self.memory_publisher = self.create_publisher(String, 'sensation', 10)
        self.current_situation = ""
        self.busy = False
        self.get_logger().info("Graph memory node started")
        # TODO: Get this hardcoding out of here
        self.db = GraphDatabase.driver("bolt://192.168.0.7:7687")
        self.execution_log = ""
        self.code = ""
        client = chromadb.Client()
        self.collection = client.create_collection(name="docs")

    def embed_node(self, node):
        # TODO: This uses the local Ollama instance
        response = ollama.embeddings(model="mxbai-embed-large", prompt=str(node))
        embedding = response["embedding"]
        self.collection.add(
            ids=[node["id"]],
            embeddings=[embedding],
            documents=[node]
        )

    def query_embeddings(self, prompt):
        # TODO: This uses the local Ollama instance
        response = ollama.embeddings(
            prompt=prompt,
            model="mxbai-embed-large"
        )
        results = self.collection.query(
            query_embeddings=[response["embedding"]],
            n_results=10
        )
        return results['documents']

    def generate_prompt(self, prompt_template: str, inputs: dict = {}):
        rv = {"system": system_message, "json": True, "prompt": prompt_template.format(**inputs)}
        related_nodes = self.query_embeddings(rv["prompt"])
        self.get_logger().info(f"Related nodes: {related_nodes}")
        rv['related_nodes'] = related_nodes
        return rv
    
    def get_entity_cypher(self, entity):
        declaration = entity['variable'] + ":" + entity['category'].replace(" ", "")
        fields = []
        for key, value in entity.items():
            if key not in ["variable", "category"]:
                fields.append(f"{key}: '{value}'")
        fields = ", ".join(fields)
        return f"MERGE ({declaration} {{{fields}}})"
    
    def get_relationship_cypher(self, relationship):
        return f"MERGE ({relationship['source']})-[:{relationship['predicate']}]->({relationship['target']})"
    
    def translate_to_cypher(self, result_as_str):
        json_string = result_as_str

        result = json.loads(json_string)
        self.get_logger().info(f"Translating to Cypher: {result}")
        if "entities" not in result:
            self.get_logger().error("No entities found in result")
            return []
        
        if "relationships" not in result:
            self.get_logger().error("No relationships found in result")
            return []
        
        entities = result["entities"]
        relationships = result["relationships"]
        
        self.get_logger().info(f"Entities: {entities}")
        self.get_logger().info(f"Relationships: {relationships}")
        
        settings = []
        if "updates" in result:
            settings = [f"SET {update['target']}.{update['field']} = '{update['value'].replace("'", "''")}'" for update in result["updates"]]
        
        blocks = [self.get_entity_cypher(entity) for entity in entities] + [self.get_relationship_cypher(relationship) for relationship in relationships] + settings + ["RETURN " + ", ".join([f"{entity['variable']}" for entity in entities])]
        
        return blocks
    
    def extract_cypher_blocks(self, result: str):
        pattern = re.compile(r"```(cypher|sql)?\s*(.*?)\s*(```|$)", re.DOTALL)
        blocks = pattern.findall(result)
        self.get_logger().info(f"Extracted {len(blocks)} blocks: {blocks}")
        blocks = [block[1] for block in blocks]
        
        single_backtick_pattern = re.compile(r"`([A-Z]+[^]]])`", re.DOTALL)
        single_backtick_blocks = single_backtick_pattern.findall(result)
        self.get_logger().info(f"Extracted {len(single_backtick_blocks)} single backtick blocks: {single_backtick_blocks}")
        
        blocks += single_backtick_blocks
        
        if blocks:
            self.execution_log += f"Extracted {len(blocks)} Cypher blocks; encoded in code fences and single backticks\n"
            return blocks
        self.execution_log += f"No code fences or single backticks found; please use markdown triple backticks or single backticks to set your code apart from your commentary\n"
        return []
    
    def run_cypher(self, cypher: str):
        self.get_logger().info(f"Running Cypher: {cypher}")
        with self.db.session() as session:
            try:
                result = session.run(cypher)
                data = result.data()
                self.get_logger().info(f"Result: {data}")
                docs = {}
                for res in data:
                    for key, value in res.items():
                        docs[key] = value
                self.get_logger().info(f"Docs: {docs}")
                for id, doc in docs.items():
                    self.embed_node(doc)
                    self.get_logger().info(f"Embedded node: {id}")                        
                return data
            except Exception as e:
                self.get_logger().error(f"Error running Cypher inner block: {e}")
                return []

    def on_result(self, result: str):
        self.get_logger().info(f"Received result: {result}")
        self.memories.append(result)
        # self.memory_publisher.publish(String(data=result))
        blocks = self.translate_to_cypher(result)
        self.get_logger().info(f"Extracted {len(blocks)} Cypher blocks {blocks}")
        self.execution_log = "Execution Log:\n"
        block = "\n".join(blocks)
        self.code = block
        if self.is_valid_cypher(block):
            self.execution_log += f"Cypher Block: {block}\n"
            try:
                result = self.run_cypher(block)
                self.execution_log += f"Result: {result}\n"
            except Exception as e:
                self.get_logger().error(f"Error running Cypher outer block: {e}")
                self.execution_log += f"Result: Error: {str(e)}\n"
        else:
            self.get_logger().warning(f"Ignored invalid Cypher block: {block}")
            self.execution_log += f"Result: Ignored invalid Cypher block\n"

        # self.memory_publisher.publish(String(data=self.execution_log))
        self.busy = False
        self.infer_situation()

    def is_valid_cypher(self, block: str):
        # A simple check to see if the block contains valid Cypher keywords
        valid_keywords = ["MATCH", "CREATE", "MERGE", "DELETE", "RETURN", "SET"]
        return any(keyword in block for keyword in valid_keywords)
        
    def situation_callback(self, msg):
        self.current_situation = msg.data
        if not self.busy:
            self.infer_situation()
    
    def infer_situation(self):
        if self.busy:
            self.get_logger().debug("Busy, skipping")
            return
        
        if not self.current_situation:
            self.get_logger().info("No situation yet")
            return
        
        self.busy = True
        history = str(self.memories) + "\n" + str(self.execution_log)
        if len(history) > 2000:
            history = history[1:1000] + "..." + history[-1000:]
             
        self.infer("""<situation>{situation}</situation>\n<execution_log>{history}<execution_log>""", {"code": self.code, "history": history, "situation": self.current_situation})
        self.memories = self.memories[-2:]
        self.execution_log = ""

def main(args=None):
    rclpy.init(args=args)
    graph_memory = GraphMemory()
    rclpy.spin(graph_memory)
    graph_memory.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

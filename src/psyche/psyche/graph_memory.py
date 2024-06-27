import rclpy
from std_msgs.msg import String
from .inference_client import InferenceClient
import re
from neo4j import GraphDatabase
import chromadb
import ollama
import json

system_message = """In well formatted JSON, list the relevant entities and their relationships as described in the natural language description of the situation. Use the following format:
{ "entities": [{"variable": "me", "category": "Person", "description": "an artificial intelligence named Pete"}, {"variable": "body", "category": "Thing", "description": "a laptop"}], "relationships": [{"source": "body", "predicate": "IS_BODY_FOR", "target": "pete"}], "updates": [{ "target": "me", "field": "name", "value": "Pete"}] }. Use only the PascalCased categories of Person, Place, Thing, Event or Idea. Capture other features in the description. As you learn more about nodes, update their fields as shown in the example. Try to capture what's in the situation faithfully. You must define all entities before you can reference them in a relationship. The variable, source, body and predicate must be single words. Use lower case for variables."""

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

    def embed_node(self, id, node):
        # TODO: This uses the local Ollama instance
        self.get_logger().info(f"Embedding node: {id} {str(node)}")
        response = ollama.embeddings(model="all-minilm", prompt=str(node))
        embedding = response["embedding"]
        self.collection.add(
            ids=[id],
            embeddings=[embedding],
            documents=[str(node)]
        )

    def query_embeddings(self, prompt):
        # TODO: This uses the local Ollama instance
        response = ollama.embeddings(
            prompt=prompt,
            model="all-minilm"
        )
        results = self.collection.query(
            query_embeddings=[response["embedding"]],
            n_results=2
        )
        self.get_logger().info(f"Results for query embeddings: {results}")
        relationships = ""
        docs = results.get('documents', [])[0]
        for docstr in docs:
            self.get_logger().info(f"Document: {docstr}")
            doc = json.loads(docstr.replace("'", "\""))
            doc_id = doc.get('id')
            if doc_id:
                self.get_logger().info(f"ID: {doc_id}")
                results = self.db.session().run("MATCH (n)-[r]->(m) WHERE ID(n) = $doc_id RETURN type(r) as type, r, ID(m) as target_id", doc_id=doc_id)
                rels = results.data()
                self.get_logger().info(f"Relationships: {rels}")
                relationships += str(rels)
        return {"docs": docs, "relationships": relationships}

    def generate_prompt(self, prompt_template: str, inputs: dict = {}):
        rv = {"system": system_message, "json": True, "prompt": prompt_template.format(**inputs)}
        related_nodes = self.query_embeddings(rv["prompt"])
        self.get_logger().info(f"Related nodes: {related_nodes}")
        rv['related_nodes'] = related_nodes
        return rv
    
    def get_entity_cypher(self, entity):
        varb = entity['variable'].replace(" ", "_")
        declaration = varb + ":" + entity['category'].replace(" ", "")
        fields = []
        for key, value in entity.items():
            escaped = value.replace("'", "\\'")
            if key not in ["variable", "category"]:
                fields.append(f"{key}: '{escaped}'")
        fields = ", ".join(fields)
        return f"MERGE ({declaration} {{{fields}}})"
    
    def get_relationship_cypher(self, relationship):
        source = relationship['source'].replace(" ", "_")
        target = relationship['target'].replace(" ", "_")
        return f"MERGE ({source})-[:{relationship['predicate']}]->({target})"
    
    def get_updates_cypher(self, update):
        target = update['target']
        key = update['field']
        value = update['value'].replace("'", "\\'")
        return f"""SET {target}.{key} = '{value}'"""
    
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
            settings = [self.get_updates_cypher(update) for update in result["updates"]]
        
        blocks = [self.get_entity_cypher(entity) for entity in entities] + [self.get_relationship_cypher(relationship) for relationship in relationships] + settings + ["RETURN " + ", ".join([f"ID({entity['variable']}), {entity['variable']}" for entity in entities])]
        
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
                        if "ID(" not in key:
                            docs[key] = {**value, **docs.get(key, {})}
                            continue
                        varb = key.split("(")[1].split(")")[0]
                        if varb not in docs:
                            docs[varb] = {}
                        docs[varb]['id'] = value
                self.get_logger().info(f"Docs: {docs}")
                for varb, doc in docs.items():
                    self.embed_node(varb, doc)
                    self.get_logger().info(f"Embedded node: {varb}")                        
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
             
        self.infer("""<situation>{situation}</situation>\n<related_existing_nodes>{related_nodes}</related_existing_nodes>\n<execution_log>{history}<execution_log>""", {"code": self.code, "history": history, "situation": self.current_situation, "related_nodes": self.query_embeddings(self.current_situation)})
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

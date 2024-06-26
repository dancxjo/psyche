import rclpy
from std_msgs.msg import String
from .inference_client import InferenceClient
import re
from neo4j import GraphDatabase

system_message = """Consider the situation below. Identify the entities that are involved in the situation. Create a list of them, dividing them into the following categories: Thing, Person, Idea, Location, Time, Context. Express these in Cypher. For each entity, use a field called description to uniquely identify it. Then list the relationships stated between them. You may also run queries to find and reference old information. Correct missing, outdated or otherwise incorrect information. As you find duplicate nodes, link them with the relationship :IS_ALSO. Remember to use the correct Cypher inside markdown syntax.
==Example==
User: I am a human named Travis. Today is une 26, 2024. I made a robot named Pete. PETE stands for "Pseudo-Conscious Experiment in Technological Evolution". Pete is currently housed in my laptop. Pete is my creation.
Assistant: The situation seems to involve "me", a Person in this case, as it is conversant. There's also a Robot named Pete, who is a person, as they are conversant. Here's the situation in Cypher.

```cypher
MERGE (me:Person {description: 'a human named Travis', name: 'Travis'})
MERGE (pete:Person {description: 'a robot named Pete'})
MERGE (acronym:Idea {description: 'Pseudo-Conscious Experiment in Technological Evolution'})
MERGE (laptop:Thing {description: 'the laptop that currently houses Pete'})
MERGE (me)-[:CREATED {before: '2024-06-26'}]->(pete)
MERGE (pete)-[:IS_NAMED_FOR]->(acronym)
MERGE (laptop)-[:HOUSES {as_of: '2024-06-26'}]->(pete)
RETURN me, pete, acronym, laptop
```
"""

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
        self.db = GraphDatabase.driver("bolt://192.168.0.7:7687")
        self.execution_log = ""

    def generate_prompt(self, prompt_template: str, inputs: dict = {}):
        return {"system": system_message, "prompt": prompt_template.format(**inputs)}
    
    def extract_cypher_blocks(self, result: str):
        pattern = re.compile(r"```(cypher)?\s*(.*?)\s*(```|$)", re.DOTALL)
        blocks = pattern.findall(result)
        self.get_logger().info(f"Extracted {len(blocks)} blocks: {blocks}")
        blocks = [block[1] for block in blocks]
        if blocks:
            self.execution_log += f"Extracted {len(blocks)} Cypher blocks; encoded in code fences\n"
            return blocks
        self.execution_log += f"No code fences found; please use markdown triple backticks to set your code apart from your commentary\n"
        return []
    
    def run_cypher(self, cypher: str):
        self.get_logger().info(f"Running Cypher: {cypher}")
        with self.db.session() as session:
            try:
                result = session.run(cypher)
                self.get_logger().info(f"Result: {result.data()}")
                return result.data()
            except Exception as e:
                self.get_logger().error(f"Error running Cypher: {e}")
                return []

    def on_result(self, result: str):
        self.get_logger().info(f"Received result: {result}")
        self.memories.append(result)
        # self.memory_publisher.publish(String(data=result))
        blocks = self.extract_cypher_blocks(result)
        self.get_logger().info(f"Extracted {len(blocks)} Cypher blocks {blocks}")
        self.execution_log = "Execution Log:\n"
        for block in blocks:
            if self.is_valid_cypher(block):
                self.execution_log += f"Cypher Block: {block}\n"
                try:
                    result = self.run_cypher(block)
                    self.execution_log += f"Result: {result}\n"
                except Exception as e:
                    self.get_logger().error(f"Error running Cypher: {e}")
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
        self.infer("""{situation}\nThe last query you ran:\n{history}\n{execution_log}""", {"history": str(self.memories), "situation": self.current_situation, "execution_log": self.execution_log })
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

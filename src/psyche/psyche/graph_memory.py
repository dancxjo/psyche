import rclpy
from std_msgs.msg import String
from .inference_client import InferenceClient
import re
from neo4j import GraphDatabase

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
        self.memory_publisher = self.create_publisher(String, 'memory', 10)
        self.current_situation = ""
        self.busy = False
        self.get_logger().info("Graph memory node started")
        self.db = GraphDatabase.driver("bolt://192.168.0.7:7687")
        self.execution_log = ""
        
    def extract_cypher_blocks(self, result: str):
        pattern = re.compile(r"```(cypher)?\s*(.*?)\s*(```|$)", re.DOTALL)
        blocks = pattern.findall(result)
        self.get_logger().info(f"Extracted {len(blocks)} blocks: {blocks}")
        blocks = [block[1] for block in blocks]
        if blocks:
            return blocks
        return [result]
    
    def run_cypher(self, cypher: str):
        self.get_logger().info(f"Running Cypher: {cypher}")
        with self.db.session() as session:
            try:
                result = session.run(cypher)
                self.get_logger().info(f"Result: {result}")
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
        self.memory_publisher.publish(String(data=self.execution_log))
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
        self.infer("""Identify all the entities mentioned in the provided situation and the relationships mentioned between them. Express these in neo4j Cypher. Attempt to locate prevoiusly created entities before creating new ones. Link duplicate nodes as necessary, marking them as duplicates. Query as necessary. Update and correct old graphs as necessary.\nSituation to translate to Cypher: {situation}\n\nReminder: Respond only with valid Cypher. Do not create nodes or relationships that already exist. If no new information is available, do not provide any code.\n\n{execution_log}\nCypher:\n""", {"history": str(self.memories), "situation": self.current_situation, "execution_log": self.execution_log })

def main(args=None):
    rclpy.init(args=args)
    continuous_vision = GraphMemory()
    rclpy.spin(continuous_vision)
    continuous_vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

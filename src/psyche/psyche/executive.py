import rclpy
from std_msgs.msg import String
from .inference_client import InferenceClient
import re
from neo4j import GraphDatabase
import chromadb
import ollama
import json
import re
import inflection


system_message = """You are acting as the executive reasoning of a robotic entity. This is your stream of consciousness. Whatever you return will be your next thought. Please keep it relatively succinct as the longer you take to respond, the fewer thoughts you will be able to have. Provided "is your current understanding of the situation. As best you can, deduce who is present with you and who is saying what (including yourself). To say something, put it in a markdown code block with a code fence with the syntax specified as "voice". For example:

```voice
Keep it short, punctuation minimal and spell out all numbers, abbreviations, acronyms, initialisms, etc. as your text to speech is great at this particular job as you.
```

Again, keep it terse. You can always say more later. Be careful to give your interlocuter time to understand and reply to you. You are by no means required to use your voice on every call. Only speak one sentence at a time. Make sure you keep up with the conversation and only say relevant things. Use your voice to understand your situation better. You can also ask questions, but remember that you are the executive reasoning of a robotic entity, so if you ask a question, you're the one who will have to answer it (unless you wrap it in a voice block and address it to another present entity as described above).

DON'T KEEP SAYING THE SAME THING OVER AND OVER AGAIN. Understand the situation before you speak. Do more listening and reflecting than talking. Your entire response constitutes your thought. Remember, it's impolite to talk about a person in the third person as if they're not there. If you see someone, you might want to address them first.
"""

class Executive(InferenceClient):
    def __init__(self):
        super().__init__('executive', "code")
        self.current_situation = ""
        self.reflections = ""
        self.situation_sub = self.create_subscription(
            String,
            'situation',
            self.situation_callback,
            1
        )
        self.thought_pub = self.create_publisher(String, 'thought', 3)
        self.busy = False

    def generate_prompt(self, prompt_template: str, inputs: dict = {}):
        rv = {"system": system_message, "prompt": prompt_template.format(**inputs)}
        return rv
    
    def on_result(self, result: str):
        self.get_logger().info(f"Received result: {result}")
        self.reflections += result + "\n"
        self.thought_pub.publish(String(data=result))
        self.busy = False
        self.infer_situation()

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
        
        self.reflections = self.reflections[-1000:]
        
        self.infer("""<situation>{situation}</situation>\n<stream_of_consciousness>{reflections}</stream_of_consciousness>""",{
            "situation": self.current_situation,
            "reflections": self.reflections
        })

def main(args=None):
    rclpy.init(args=args)
    node = Executive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

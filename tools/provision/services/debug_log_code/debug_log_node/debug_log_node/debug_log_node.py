import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading


class DebugLogNode(Node):
    def __init__(self):
        super().__init__('debug_log_node')
        self.pub = self.create_publisher(String, '/voice', 10)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._follow_journal, daemon=True)
        self._thread.start()

    def _follow_journal(self):
        # Use journalctl --follow --output=short
        proc = subprocess.Popen(['journalctl', '--follow', '--output=short'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        try:
            while rclpy.ok() and not self._stop.is_set():
                line = proc.stdout.readline()
                if not line:
                    break
                text = line.decode(errors='ignore').strip()
                if text:
                    msg = String()
                    msg.data = text
                    self.pub.publish(msg)
        except Exception:
            pass
        finally:
            try:
                proc.terminate()
            except Exception:
                pass

    def destroy_node(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DebugLogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import queue
import subprocess
import shutil
import os


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        self.q = queue.Queue()
        self.speak_lock = threading.Lock()

        self.declare_parameter('piper_cmd', 'piper')
        self.declare_parameter('voice', 'en-us')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('mbrola_voice', 'en1')

        self.piper_cmd = self.get_parameter('piper_cmd').get_parameter_value().string_value
        self.voice = self.get_parameter('voice').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.mbrola_voice = self.get_parameter('mbrola_voice').get_parameter_value().string_value

        self.subscription = self.create_subscription(String, '/voice', self.on_voice, 10)
        self._stop = threading.Event()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

    def on_voice(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'Queued voice: {text[:40]}')
        self.q.put(text)

    def _speak_with_piper(self, text: str):
        # piper usage example (from coqui TTS/piper): piper -l en-US -t "text" -o out.wav or play directly
        # Here we try to run piper to stdout or produce an audio file and play with aplay
        try:
            # If piper binary supports TTS direct play, invoke it. Otherwise, render wav and play.
            piper_path = shutil.which(self.piper_cmd)
            if piper_path:
                # Render to temp wav then aplay
                tmp = '/tmp/psyched_voice.wav'
                cmd = [piper_path, '-l', self.voice, '-t', text, '-o', tmp]
                subprocess.run(cmd, check=True)
                subprocess.run(['aplay', tmp], check=True)
                try:
                    os.remove(tmp)
                except Exception:
                    pass
                return True
        except Exception:
            self.get_logger().warning('piper failed')
        return False

    def _speak_with_mbrola(self, text: str):
        # mbrola usage: echo "text" | mbrola [voice] - | aplay
        mbrola_path = shutil.which('mbrola')
        if not mbrola_path:
            return False
        try:
            # Using subprocess pipeline: echo text | text2pho ? but simplest: use espeak-ng with mbrola output
            # We will try: espeak-ng -v mbrola-<voice> -w /tmp/out.wav "text" then aplay
            tmp = '/tmp/psyched_voice_mbrola.wav'
            cmd = ['espeak-ng', '-v', f'mbrola-{self.mbrola_voice}', '-w', tmp, text]
            subprocess.run(cmd, check=True)
            subprocess.run(['aplay', tmp], check=True)
            try:
                os.remove(tmp)
            except Exception:
                pass
            return True
        except Exception:
            return False

    def _speak_with_espeak(self, text: str):
        # Try espeak-ng, then espeak
        for cmd in (['espeak-ng', text], ['espeak', text]):
            try:
                subprocess.run(cmd, check=True)
                return True
            except Exception:
                continue
        return False

    def _worker_loop(self):
        while rclpy.ok() and not self._stop.is_set():
            try:
                text = self.q.get(timeout=0.5)
            except queue.Empty:
                continue

            with self.speak_lock:
                self.get_logger().info(f'Speaking: {text[:80]}')
                ok = self._speak_with_piper(text)
                if not ok:
                    self.get_logger().info('Falling back to espeak')
                    self._speak_with_espeak(text)
                self.q.task_done()

    def destroy_node(self):
        self._stop.set()
        self.worker.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

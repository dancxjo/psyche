import threading
import argparse
import struct
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData

try:
    import sounddevice as sd
except Exception:
    sd = None

try:
    import webrtcvad
except Exception:
    webrtcvad = None


class MicNode(Node):
    def __init__(self, name='mic_node'):
        super().__init__(name)

        # ROS parameters
        self.declare_parameter('device', '')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_ms', 30)
        self.declare_parameter('channels', 1)
        self.declare_parameter('vad_aggressiveness', 2)

        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.frame_ms = self.get_parameter('frame_ms').get_parameter_value().integer_value
        self.channels = self.get_parameter('channels').get_parameter_value().integer_value
        self.vad_aggressiveness = self.get_parameter('vad_aggressiveness').get_parameter_value().integer_value

        self.audio_pub = self.create_publisher(AudioData, 'audio/pcm', 10)
        self.vad_pub = self.create_publisher(Bool, 'mic/vad', 10)

        self._vad = None
        if webrtcvad is not None:
            try:
                self._vad = webrtcvad.Vad(self.vad_aggressiveness)
            except Exception:
                self._vad = None

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        if sd is None:
            self.get_logger().error('sounddevice not available; cannot capture audio')
            return

    frame_size = int(self.sample_rate * (self.frame_ms / 1000.0))

        try:
            with sd.InputStream(samplerate=self.sample_rate, channels=self.channels, dtype='int16') as stream:
                while rclpy.ok() and not self._stop.is_set():
                    frames, overflowed = stream.read(frame_size)
                    if overflowed:
                        self.get_logger().warn('Input overflow')

                    # frames is shape (frame_size, channels); convert to bytes
                    if frames is None:
                        time.sleep(self.frame_ms / 1000.0)
                        continue

                    # If channels >1 flatten interleaved
                    if self.channels > 1:
                        frames = frames.flatten()

                    # pack int16 to bytes
                    data = frames.tobytes()

                    # publish as audio_common_msgs/AudioData
                    audio_msg = AudioData()
                    audio_msg.data = list(data)
                    self.audio_pub.publish(audio_msg)

                    # VAD
                    vad_state = False
                    if self._vad is not None:
                        try:
                            # webrtcvad expects 16-bit mono PCM bytes
                            is_speech = self._vad.is_speech(data, sample_rate=self.sample_rate)
                            vad_state = bool(is_speech)
                        except Exception:
                            vad_state = False

                    self.vad_pub.publish(Bool(data=vad_state))

        except Exception as e:
            self.get_logger().error(f'Audio capture error: {e}')

    def destroy_node(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('--sample-rate', type=int, default=16000)
    parser.add_argument('--frame-ms', type=int, default=30)
    parser.add_argument('--channels', type=int, default=1)
    args, _ = parser.parse_known_args()

    node = MicNode(sample_rate=args.sample_rate, frame_ms=args.frame_ms, channels=args.channels)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

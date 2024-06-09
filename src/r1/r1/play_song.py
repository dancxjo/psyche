import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from create_msgs.msg import DefineSong, PlaySong

class SongPlayer(Node):
    def __init__(self):
        super().__init__('song_player')
        self.subscription = self.create_subscription(
            String,
            'song',
            self.song_callback,
            10
        )
        self.publisher_define_song = self.create_publisher(
            DefineSong,
            'define_song',
            10
        )
        self.publisher_play_song = self.create_publisher(
            PlaySong,
            'play_song',
            10
        )

    def song_callback(self, msg):
        self.get_logger().debug('Received song: %s' % msg.data)
        # Process the received song here
        def process_song(self, song):
            try:
                song_array = json.loads(song)
                if len(song_array) < 1 or len(song_array) > 16:
                    self.get_logger().error('Invalid song length: %d' % len(song_array))
                    return

                for note in song_array:
                    if not isinstance(note, dict):
                        self.get_logger().error('Invalid note format: %s' % note)
                        return

                    if 'note' not in note or 'duration' not in note:
                        self.get_logger().error('Invalid note format: %s' % note)
                        return

                    if not isinstance(note['note'], int) or note['note'] < 31 or note['note'] > 127:
                        self.get_logger().error('Invalid note value: %s' % note['note'])
                        return

                    if not isinstance(note['duration'], float) or note['duration'] <= 0:
                        self.get_logger().error('Invalid note duration: %s' % note['duration'])
                        return

                    song_msg = DefineSong()
                    song_msg.song = 0
                    song_msg.length = len(song_array)
                    song_msg.notes = [note['note'] for note in song_array]
                    song_msg.durations = [note['duration'] for note in song_array]
                    self.publisher_define_song.publish(song_msg)
                    
                    play_song_msg = PlaySong()
                    play_song_msg.song = 0
                    self.publisher_play_song.publish(play_song_msg)

            except json.JSONDecodeError:
                self.get_logger().error('Invalid song format: %s' % song)
                return

        # Call the process_song function in song_callback
        process_song(self, msg.data)

        play_it_sam = PlaySong(song=0)
        # Publish the song to be played
        self.publisher_play_song.publish(play_it_sam)

def main(args=None):
    rclpy.init(args=args)
    song_player = SongPlayer()
    rclpy.spin(song_player)
    song_player.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
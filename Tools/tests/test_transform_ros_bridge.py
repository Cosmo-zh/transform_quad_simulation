"""Image payload integrity and optional live Gazebo -> ROS delivery."""
import os
os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from transform_stereo_ros_bridge import convert_image

try:
    from gz.msgs10.image_pb2 import Image as GzImage
    from sensor_msgs.msg import Image
    import rclpy
    AVAILABLE = True
except ImportError:
    AVAILABLE = False


@unittest.skipUnless(AVAILABLE, 'ROS 2 and Gazebo Python bindings required')
class ImageBridgeTest(unittest.TestCase):
    def source(self, pixel_format=3):
        message = GzImage()
        message.width, message.height, message.step = 2, 1, 8
        message.pixel_format_type = pixel_format
        message.data = bytes([255, 0, 0, 0, 255, 0, 7, 8])
        message.header.stamp.sec, message.header.stamp.nsec = 42, 123456
        return message

    def test_payload_padding_timestamp_and_encoding(self):
        for pixel_format, encoding in [(3, 'rgb8'), (8, 'bgr8')]:
            source = self.source(pixel_format)
            image = convert_image(source, 'left_optical')
            self.assertEqual(bytes(image.data), source.data)
            self.assertEqual((image.width, image.height, image.step), (2, 1, 8))
            self.assertEqual(image.encoding, encoding)
            self.assertEqual((image.header.stamp.sec, image.header.stamp.nanosec), (42, 123456))
            self.assertEqual(image.header.frame_id, 'left_optical')

    def test_invalid_format_and_payload_rejected(self):
        with self.assertRaises(ValueError):
            convert_image(self.source(1), 'left')
        source = self.source()
        source.data = b'bad'
        with self.assertRaises(ValueError):
            convert_image(source, 'left')

    def test_preview_downsampling_preserves_selected_pixels(self):
        source = self.source()
        image = convert_image(source, 'left', scale=2)
        self.assertEqual((image.width, image.height, image.step), (1, 1, 3))
        self.assertEqual(bytes(image.data), source.data[:3])
        self.assertEqual(image.header.stamp.sec, 42)

    @unittest.skipUnless(os.environ.get('RUN_STEREO_ROS_INTEGRATION') == '1', 'requires local network access')
    def test_live_stereo_delivery(self):
        from rclpy.qos import qos_profile_sensor_data
        with tempfile.TemporaryDirectory() as temporary:
            env = os.environ.copy()
            partition = 'test_stereo_'+uuid.uuid4().hex
            env['GZ_PARTITION'] = partition
            env['ROS_DOMAIN_ID'] = str(100+uuid.uuid4().int % 100)
            os.environ['GZ_PARTITION'] = partition
            from gz.transport13 import Node
            stats = Path(temporary) / 'stats.json'
            log = (Path(temporary) / 'bridge.txt').open('w')
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve().parents[1] /
                                        'transform_stereo_ros_bridge.py'), '--output', str(stats), '--scale', '1'],
                                       env=env, stdout=log, stderr=subprocess.STDOUT)
            rclpy.init(domain_id=int(env['ROS_DOMAIN_ID']))
            ros = rclpy.create_node('test_stereo_receiver')
            received = {}
            transport = Node()
            publishers = {}
            for side in ['left', 'right']:
                topic = f'/transform/stereo/{side}/image'
                publishers[side] = transport.advertise(topic, GzImage)
                ros.create_subscription(Image, topic, lambda m, side=side: received.update({side: m}),
                                        qos_profile_sensor_data)
            try:
                deadline = time.monotonic()+20
                while time.monotonic() < deadline and len(received) < 2:
                    for publisher in publishers.values():
                        publisher.publish(self.source())
                    rclpy.spin_once(ros, timeout_sec=.1)
                    self.assertIsNone(process.poll(), 'bridge exited')
                diagnostic = stats.read_text() if stats.exists() else 'no stats'
                self.assertEqual(set(received), {'left', 'right'}, diagnostic)
                for side, image in received.items():
                    self.assertEqual(bytes(image.data), self.source().data)
                    self.assertEqual(image.header.frame_id, f'stereo_{side}_optical_frame')
            finally:
                process.send_signal(2)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
                ros.destroy_node()
                rclpy.shutdown()
                log.close()
            recorded = json.loads(stats.read_text())
            self.assertTrue(all(count > 0 for count in recorded['published'].values()))


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Forward raw Gazebo stereo images to ROS 2, without flight-control inputs."""
import argparse
import json
import os
from pathlib import Path
import threading

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')


def convert_image(source, frame_id, scale=1):
    from sensor_msgs.msg import Image
    encodings = {3: 'rgb8', 8: 'bgr8'}
    if source.pixel_format_type not in encodings:
        raise ValueError(f'Unsupported Gazebo pixel format: {source.pixel_format_type}')
    if source.step < source.width*3 or len(source.data) != source.height*source.step:
        raise ValueError('Invalid image dimensions or payload')
    image = Image()
    image.header.stamp.sec = source.header.stamp.sec
    image.header.stamp.nanosec = source.header.stamp.nsec
    image.header.frame_id = frame_id
    image.height, image.width = source.height, source.width
    image.encoding = encodings[source.pixel_format_type]
    image.is_bigendian = 0
    image.step = source.step
    if scale == 1:
        image.data = source.data
    else:
        import numpy as np
        pixels = np.frombuffer(source.data, dtype=np.uint8).reshape(source.height, source.step)
        pixels = pixels[:, :source.width*3].reshape(source.height, source.width, 3)
        small = np.ascontiguousarray(pixels[::scale, ::scale])
        image.height, image.width = small.shape[:2]
        image.step = image.width*3
        image.data = small.tobytes()
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=float, default=5)
    parser.add_argument('--scale', type=int, choices=[1, 2, 4], default=2)
    args = parser.parse_args()
    if not 0 < args.fps <= 15:
        parser.error('--fps must be between 0 and 15')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image as GzImage

    rclpy.init()
    ros = rclpy.create_node('transform_stereo_images')
    transport = Node()
    lock = threading.Lock()
    latest = {}
    counts = dict(left=0, right=0)
    subscribers = dict(left=0, right=0)
    peak_subscribers = dict(left=0, right=0)
    callbacks = []
    publishers = {}
    for side in counts:
        topic = f'/transform/stereo/{side}/image'
        publishers[side] = ros.create_publisher(Image, topic, qos_profile_sensor_data)

        def receive(message, side=side):
            # Bounded latest-frame slot: rendering must never back up the simulator.
            with lock:
                latest[side] = message

        callbacks.append(receive)
        if not transport.subscribe(GzImage, topic, receive):
            raise RuntimeError(f'Could not subscribe to {topic}')

    def publish_latest():
        with lock:
            pending = latest.copy()
            latest.clear()
        for side, source in pending.items():
            try:
                image = convert_image(source, f'stereo_{side}_optical_frame', args.scale)
            except ValueError as exc:
                ros.get_logger().error(str(exc))
                continue
            publishers[side].publish(image)
            counts[side] += 1

    def save_stats():
        if rclpy.ok():
            subscribers.update({side: pub.get_subscription_count() for side, pub in publishers.items()})
            for side in counts:
                peak_subscribers[side] = max(peak_subscribers[side], subscribers[side])
        stats = dict(partition=os.environ.get('GZ_PARTITION'), published=counts,
                     subscribers=subscribers, peak_subscribers=peak_subscribers,
                     fps_limit=args.fps, scale=args.scale)
        temporary = args.output.with_suffix('.tmp')
        temporary.write_text(json.dumps(stats, indent=2)+'\n')
        temporary.replace(args.output)

    ros.create_timer(1/args.fps, publish_latest)
    ros.create_timer(1.0, save_stats)
    ros.get_logger().info('Stereo bridge ready: Gazebo images -> ROS Image (sensor-data QoS)')
    try:
        rclpy.spin(ros)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # The periodic snapshot also survives a forced process shutdown.
        save_stats()
        ros.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

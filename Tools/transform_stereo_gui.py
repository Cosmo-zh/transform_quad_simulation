#!/usr/bin/env python3
"""Send bounded camera-follow requests to the Gazebo GUI in this partition."""
import argparse
import os
import time

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
from gz.transport13 import Node
from gz.msgs10.cameratrack_pb2 import CameraTrack


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='transform_quad_plus_stereo_0')
    args = parser.parse_args()
    node = Node()
    publisher = node.advertise('/gui/track', CameraTrack)
    message = CameraTrack()
    message.track_mode = CameraTrack.FOLLOW
    message.follow_target.name = args.model
    message.follow_offset.x, message.follow_offset.y, message.follow_offset.z = -2.5, -2, 1.8
    message.follow_pgain = message.track_pgain = 1.0
    connected = False
    sent = 0
    for _ in range(300):
        if publisher.has_connections():
            connected = True
            publisher.publish(message)
            sent += 1
            if sent >= 10:
                break
        time.sleep(.1)
    print(f'Camera follow: model={args.model}, subscriber={connected}', flush=True)
    return 0 if connected else 1


if __name__ == '__main__':
    raise SystemExit(main())

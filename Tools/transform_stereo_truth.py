#!/usr/bin/env python3
"""Independent Gazebo truth recorder. No data is sent to the flight controller."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output)
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    node = Node()
    with (output / 'ground_truth.jsonl').open('w') as log:
        def callback(msg):
            for pose in msg.pose:
                if pose.name == 'transform_quad_plus_stereo_0':
                    p, q = pose.position, pose.orientation
                    record = dict(stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,
                                  position=[p.x, p.y, p.z], quaternion=[q.w, q.x, q.y, q.z])
                    log.write(json.dumps(record)+'\n')
                    log.flush()
        node.subscribe(Pose_V, '/world/transform_stereo_passage/pose/info', callback)
        while running:
            time.sleep(.1)


if __name__ == '__main__':
    main()

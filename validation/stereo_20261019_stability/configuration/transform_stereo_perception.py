#!/usr/bin/env python3
"""Rectified ideal stereo gate measurements; no world poses or gate widths used.

The first detector uses the passage world's turquoise vertical edge strips.
It is deliberately a marked-gate prototype, not a general doorway detector.
"""
import math
import os
from collections import deque
from dataclasses import asdict, dataclass
from functools import lru_cache
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
import cv2
import numpy as np

# A small stereo detector must not spawn one OpenCV worker per host CPU while
# Gazebo, PX4 and two viewers compete for the same machine.
cv2.setNumThreads(1)

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'Tools/simulation/gz/models/transform_quad_plus_stereo/model.sdf'


@dataclass(frozen=True)
class Calibration:
    width: int = 960
    height: int = 600
    hfov: float = 1.2
    baseline: float = .12

    @property
    def focal(self):
        return self.width / (2 * math.tan(self.hfov / 2))


@dataclass(frozen=True)
class Gate:
    stamp: float
    width: float
    height: float
    forward: float
    right: float
    down: float
    normal_yaw: float
    uncertainty: float
    full_height: bool

    @property
    def lateral_error(self):
        """Body-center offset along the gate plane, including the 8 cm camera mount."""
        return self.right*math.cos(self.normal_yaw)-(self.forward+.08)*math.sin(self.normal_yaw)

    def as_dict(self):
        return dict(asdict(self), lateral_error=self.lateral_error)


class GateDetector:
    def __init__(self, calibration=Calibration()):
        self.calibration = calibration

    def strips(self, rgb, allow_clipped=False):
        if rgb.shape != (self.calibration.height, self.calibration.width, 3):
            raise ValueError('Image size differs from calibration')
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, np.array([65, 85, 45]), np.array([100, 255, 255]))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        strips = []
        for label in range(1, count):
            x, y, w, h, area = stats[label]
            if h < 35 or w < 2 or h < 4*w or area < h or x < 3 or x+w >= rgb.shape[1]-3:
                continue
            if not allow_clipped and (y < 3 or y+h >= rgb.shape[0]-3):
                continue
            # Reject severe roll/perspective: this prototype expects near-level approach.
            rows, cols = np.where(labels == label)
            slope = np.polyfit(rows, cols, 1)[0]
            if abs(slope) > math.tan(math.radians(5)):
                continue
            strips.append((x, y, w, h, label, labels))
        return strips

    def pair(self, rgb, allow_clipped=False):
        strips = self.strips(rgb, allow_clipped)
        pairs = []
        for a in strips:
            for b in strips:
                if a[0] >= b[0] or b[0] - a[0] < 20:
                    continue
                if abs(a[1]-b[1]) > .08*max(a[3], b[3]) or abs(a[3]-b[3]) > .12*max(a[3], b[3]):
                    continue
                pairs.append((min(a[3], b[3]), a, b))
        if not pairs:
            return None
        _, left, right = max(pairs, key=lambda item: item[0])
        top = max(left[1], right[1])
        bottom = min(left[1]+left[3], right[1]+right[3])-1
        values = []
        for strip, inner_right in [(left, True), (right, False)]:
            edge = []
            for row in range(top+3, bottom-2):
                xs = np.flatnonzero(strip[5][row] == strip[4])
                if xs.size:
                    # Pixel boundary rather than center: measure clear aperture.
                    edge.append(xs[-1]+.5 if inner_right else xs[0]-.5)
            values.append(float(np.mean(edge)))
        return values, top-.5, bottom+.5

    def detect(self, left, right, stamp=0.0, allow_clipped=False):
        a, b = self.pair(left, allow_clipped), self.pair(right, allow_clipped)
        if a is None or b is None:
            return None
        if max(abs(a[1]-b[1]), abs(a[2]-b[2])) > 3:
            return None
        c = self.calibration
        disparity = np.asarray(a[0]) - np.asarray(b[0])
        if np.any(disparity < 3) or np.any(disparity > c.width/3):
            return None
        depth = c.focal*c.baseline/disparity
        # At long range one pixel of edge quantization can imply a large false
        # plane angle. Pool indistinguishable disparities; refine at the stop.
        if abs(disparity[0]-disparity[1]) <= 1 and float(np.mean(depth)) > 1.0:
            depth[:] = c.focal*c.baseline/float(np.mean(disparity))
        x = (np.asarray(a[0])-(c.width-1)/2)*depth/c.focal-c.baseline/2
        width = float(x[1]-x[0])
        yaw = -math.atan2(float(depth[1]-depth[0]), width)
        z = float(np.mean(depth))
        if not .2 < width < 2 or not .3 < z < 8 or abs(yaw) > math.radians(8):
            return None
        return Gate(stamp, width, (a[2]-a[1])*z/c.focal, z, float(np.mean(x)),
                    ((a[1]+a[2])/2-(c.height-1)/2)*z/c.focal, yaw,
                    max(.004, width*.5/float(np.min(disparity))),
                    bool(min(a[1], b[1]) > 2 and max(a[2], b[2]) < c.height-3))


def fit_heading_offset(positions, relative_vectors):
    """Align visual displacement with local odometry, without world heading.

    For one stationary gate: p_i + R(offset) q_i = constant. The 2D
    least-squares rotation compensates estimator heading/frame disagreement.
    """
    if len(positions) < 30:
        return None
    p, q = np.asarray(positions)[:, :2], np.asarray(relative_vectors)[:, :2]
    p, q = p-p.mean(axis=0), q-q.mean(axis=0)
    if np.max(np.linalg.norm(p, axis=1)) < .35:
        return None
    a = -q
    angle = math.atan2(float(np.sum(a[:, 0]*p[:, 1]-a[:, 1]*p[:, 0])), float(np.sum(a*p)))
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    residual = float(np.sqrt(np.mean(np.sum((a @ rotation.T-p)**2, axis=1))))
    if abs(angle) > math.radians(20) or residual > .04:
        return None
    return angle, residual


class StereoStream:
    """Exact simulation-time stereo pairing; bounded buffers, no repeated frames."""
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        self.node = Node()
        self.lock = threading.Lock()
        self.frames = {'left': deque(maxlen=5), 'right': deque(maxlen=5)}
        self.callbacks = []
        for side in self.frames:
            def callback(msg, side=side):
                # gz.msgs.PixelFormatType.RGB_INT8 == 3, BGR_INT8 == 8.
                if msg.pixel_format_type not in (3, 8):
                    return
                stamp = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9
                data = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
                rgb = data[:, :msg.width*3].reshape(msg.height, msg.width, 3).copy()
                if msg.pixel_format_type == 8:
                    rgb = rgb[:, :, ::-1].copy()
                with self.lock:
                    self.frames[side].append((stamp, time.monotonic(), rgb))
            self.callbacks.append(callback)
            if not self.node.subscribe(Image, f'/transform/stereo/{side}/image', callback):
                raise RuntimeError(f'Could not subscribe to {side} camera')

    def next_pair(self):
        with self.lock:
            for left in reversed(self.frames['left']):
                for right in reversed(self.frames['right']):
                    if abs(left[0]-right[0]) < 1e-6 and time.monotonic()-min(left[1], right[1]) < 1:
                        self.frames['left'].clear()
                        self.frames['right'].clear()
                        return left[0], left[2], right[2]
        return None


@lru_cache(maxsize=1)
def rotor_geometry():
    model = ET.parse(MODEL).getroot().find('model')
    geometry = []
    for i in range(1, 5):
        hinge = list(map(float, model.find(f"joint[@name='arm{i}_Joint']/pose").text.split()))
        fan = list(map(float, model.find(f"joint[@name='fan{i}_Joint']/pose").text.split()))
        geometry.append((hinge, fan))
    return geometry


def envelope(progress, yaw=0.0):
    """Swept disks from actual SDF hinge/fan transforms, body FLU geometry."""
    points = []
    for hinge, fan in rotor_geometry():
        angle = hinge[5]-math.pi/3*progress
        x = hinge[0]+math.cos(angle)*fan[0]-math.sin(angle)*fan[1]
        y = hinge[1]+math.sin(angle)*fan[0]+math.cos(angle)*fan[1]
        points.append((math.cos(yaw)*x-math.sin(yaw)*y, math.sin(yaw)*x+math.cos(yaw)*y))
    return max(y for x, y in points)-min(y for x, y in points)+.1271


def required_progress(width, margin=.04, uncertainty=0.0, max_progress=1.0):
    if not all(math.isfinite(v) for v in (width, margin, uncertainty, max_progress)):
        raise ValueError('Non-finite planning input')
    if margin < 0 or uncertainty < 0 or not 0 <= max_progress <= 1:
        raise ValueError('Invalid planning bounds')
    available = width-2*margin-2*uncertainty
    # Explicit yaw allowance; do not assume perfect alignment.
    for progress in np.linspace(0, max_progress, 201):
        if max(envelope(float(progress), float(y)) for y in np.linspace(-math.radians(5), math.radians(5), 11)) <= available:
            return float(progress)
    return None


def lateral_allowance(width, progress, uncertainty):
    swept_width = max(envelope(progress, float(y)) for y in np.linspace(-math.radians(5), math.radians(5), 11))
    return (width-swept_width)/2-uncertainty

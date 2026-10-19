"""Independent projection fixtures test stereo scale/sign and rejection paths."""
import math
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2
import numpy as np
from transform_stereo_perception import Calibration, Gate, GateDetector, MODEL, envelope, required_progress, fit_heading_offset, lateral_allowance


def render(width, distance, center_right=0, center_down=0):
    c = Calibration()
    images = []
    for camera_right in [-c.baseline/2, c.baseline/2]:
        image = np.zeros((c.height, c.width, 3), np.uint8)
        for side in [-1, 1]:
            xs = [side*width/2, side*(width/2+.016)]
            us = [round((c.width-1)/2+c.focal*(x+center_right-camera_right)/distance) for x in xs]
            vs = [round((c.height-1)/2+c.focal*(y+center_down)/distance) for y in [-.35, .35]]
            cv2.rectangle(image, (min(us), min(vs)), (max(us), max(vs)), (13, 217, 184), -1)
        images.append(image)
    return images


class StereoTest(unittest.TestCase):
    def test_gate_plane_offset_with_yawed_camera(self):
        for yaw in [math.radians(-5), 0, math.radians(5)]:
            for distance in [.5, .8, 1.4]:
                for offset in [-.035, .035]:
                    # Independent plane basis: body-center -> gate center is
                    # distance * normal + offset * tangent, before camera mount.
                    forward = distance*math.cos(yaw)-offset*math.sin(yaw)-.08
                    right = distance*math.sin(yaw)+offset*math.cos(yaw)
                    gate = Gate(0, .44, .7, forward, right, 0, yaw, .004, True)
                    self.assertAlmostEqual(gate.lateral_error, offset, places=9)

    def test_centered_image_can_hide_unsafe_body_offset(self):
        distance, offset = .6, .035
        yaw = -math.atan2(offset, distance)
        forward = distance*math.cos(yaw)-offset*math.sin(yaw)-.08
        right = distance*math.sin(yaw)+offset*math.cos(yaw)
        gate = Gate(0, .44, .7, forward, right, 0, yaw, .004, True)
        self.assertAlmostEqual(gate.right, 0, places=9)
        self.assertAlmostEqual(gate.lateral_error, .035, places=9)

    def test_heading_alignment(self):
        positions = np.array([[0, t, 0] for t in np.linspace(0, 1.2, 100)])
        bias = .105
        rotation = np.array([[math.cos(bias), -math.sin(bias)], [math.sin(bias), math.cos(bias)]])
        relatives = np.zeros_like(positions)
        relatives[:, :2] = (np.array([0, 3])-positions[:, :2]) @ rotation.T
        offset, residual = fit_heading_offset(positions, relatives)
        self.assertAlmostEqual(offset, -bias, places=6)
        self.assertLess(residual, 1e-6)
        self.assertIsNone(fit_heading_offset(positions[:5], relatives[:5]))

    def test_projected_measurements(self):
        detector = GateDetector()
        for width, depth, right, down in [(.6, 2.8, 0, 0), (.44, 1.8, .08, -.03), (.6, 3.2, -.1, .04)]:
            with self.subTest(width=width, depth=depth):
                gate = detector.detect(*render(width, depth, right, down))
                self.assertIsNotNone(gate)
                json.dumps(gate.as_dict())
                self.assertAlmostEqual(gate.width, width, delta=.02)
                self.assertAlmostEqual(gate.forward, depth, delta=.1)
                self.assertAlmostEqual(gate.right, right, delta=.01)
                self.assertAlmostEqual(gate.down, down, delta=.01)

    def test_invalid_images(self):
        detector = GateDetector()
        left, right = render(.6, 2.8)
        self.assertIsNone(detector.detect(left*0, right))
        self.assertIsNone(detector.detect(left, left))
        self.assertIsNone(detector.detect(right, left))
        self.assertIsNone(detector.detect(*render(.6, .5)))  # Clipped gate edges.

    def test_clipped_strips_only_supply_horizontal_measurement(self):
        detector = GateDetector()
        pair = render(.44, .6)
        self.assertIsNone(detector.detect(*pair))
        gate = detector.detect(*pair, allow_clipped=True)
        self.assertIsNotNone(gate)
        self.assertFalse(gate.full_height)
        self.assertAlmostEqual(gate.width, .44, delta=.01)

    def test_planning_clearance(self):
        self.assertEqual(required_progress(.6), 0)
        progress = required_progress(.44, uncertainty=.005)
        self.assertGreater(progress, .8)
        self.assertIsNone(required_progress(.36))
        self.assertIsNone(required_progress(.44, max_progress=.3))
        self.assertLessEqual(envelope(progress)+.08+.01, .44)
        self.assertGreater(lateral_allowance(.6, 0, .005), .08)
        self.assertGreaterEqual(lateral_allowance(.44, progress, .005), .04)
        with self.assertRaises(ValueError):
            required_progress(float('nan'))

    def test_calibration_matches_generated_model(self):
        sensors = ET.parse(MODEL).getroot().find('model/link').findall("sensor[@type='camera']")
        self.assertEqual(len(sensors), 2)
        c = Calibration()
        poses = [list(map(float, s.find('pose').text.split())) for s in sensors]
        self.assertAlmostEqual(poses[0][1]-poses[1][1], c.baseline)
        for sensor in sensors:
            self.assertEqual(int(sensor.find('camera/image/width').text), c.width)
            self.assertEqual(int(sensor.find('camera/image/height').text), c.height)
            self.assertAlmostEqual(float(sensor.find('camera/horizontal_fov').text), c.hfov)


if __name__ == '__main__':
    unittest.main()

"""Control must reject old perception and never wait for image processing."""
import queue
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from transform_stereo_vision_worker import VisionWorker


class VisionWorkerTest(unittest.TestCase):
    def worker(self):
        worker = VisionWorker.__new__(VisionWorker)
        worker.output = queue.Queue(maxsize=2)
        worker.clipped = threading.Event()
        worker.process = mock.Mock()
        worker.process.is_alive.return_value = True
        return worker

    def test_only_latest_fresh_measurement_reaches_control(self):
        worker = self.worker()
        worker.output.put(dict(gate='old', produced=time.monotonic()))
        worker.output.put(dict(gate='new', produced=time.monotonic()))
        self.assertEqual(worker.next_gate(True), 'new')
        self.assertTrue(worker.clipped.is_set())
        self.assertIsNone(worker.next_gate(False))
        self.assertFalse(worker.clipped.is_set())

    def test_stale_perception_is_discarded(self):
        worker = self.worker()
        worker.output.put(dict(gate='old', produced=time.monotonic()-2))
        self.assertIsNone(worker.next_gate())

    def test_worker_failure_cannot_silently_continue_flight(self):
        worker = self.worker()
        worker.output.put(dict(error='Disk full'))
        with self.assertRaisesRegex(RuntimeError, 'Disk full'):
            worker.next_gate()
        worker.process.is_alive.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'exited'):
            worker.next_gate()

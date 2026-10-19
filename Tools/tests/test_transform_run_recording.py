"""Recording must survive startup failures and export unambiguous coordinates."""
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import transform_stereo_sitl_check as runner


class RecordingTest(unittest.TestCase):
    def test_second_run_is_rejected_until_first_releases_lock(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'active.lock'
            with runner.acquire_run_lock(path):
                with self.assertRaisesRegex(RuntimeError, 'Another stereo run'):
                    runner.acquire_run_lock(path)
            with runner.acquire_run_lock(path):
                pass

    def test_missing_ros_setup_does_not_start_flight(self):
        with tempfile.TemporaryDirectory() as temporary:
            with mock.patch.object(sys, 'argv', ['runner', '--flight', '--rviz', '--output-root', temporary,
                                                 '--ros-setup', str(Path(temporary) / 'missing.bash')]), \
                    mock.patch.dict(os.environ, {'DISPLAY': ':0'}), mock.patch('pexpect.spawn') as spawn:
                self.assertEqual(runner.main(), 1)
                spawn.assert_not_called()
            run = next(Path(temporary).iterdir())
            result = json.loads((run / 'result.json').read_text())
            self.assertFalse(result['success'])
            self.assertIn('ROS setup not found', result['error'])
            self.assertTrue((run / 'manifest.json').exists())

    def test_analysis_failure_keeps_raw_data_and_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / 'result.json').write_text('{"success": false}')
            (run / 'events.jsonl').write_text('raw recording\n')
            metadata = {}
            with mock.patch.object(runner.subprocess, 'run', side_effect=OSError('analysis unavailable')):
                runner.finish_run(run, metadata, os.environ.copy(), False)
            self.assertFalse(metadata['analysis_success'])
            self.assertEqual((run / 'events.jsonl').read_text(), 'raw recording\n')
            self.assertTrue((run / 'manifest.json').exists())

    def test_startup_failure_still_saves_result_and_manifest(self):
        with tempfile.TemporaryDirectory() as temporary:
            with mock.patch.object(sys, 'argv', ['runner', '--flight', '--output-root', temporary]), \
                    mock.patch('pexpect.spawn', side_effect=OSError('test startup failure')):
                self.assertEqual(runner.main(), 1)
            run = next(Path(temporary).iterdir())
            self.assertFalse(json.loads((run / 'result.json').read_text())['success'])
            self.assertIn('test startup failure', (run / 'result.json').read_text())
            metadata = json.loads((run / 'run.json').read_text())
            self.assertEqual(metadata['exit_code'], 1)
            self.assertTrue(metadata['analysis_success'])
            manifest = json.loads((run / 'manifest.json').read_text())
            self.assertIn('analysis/measurements.csv', [entry['path'] for entry in manifest])
            for entry in manifest:
                self.assertEqual(hashlib.sha256((run / entry['path']).read_bytes()).hexdigest(), entry['sha256'])

    def test_csv_preserves_ned_and_world_coordinates(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / 'result.json').write_text('{"success": false, "error": "interrupted"}')
            records = [dict(event='measurement', time=1, stamp=2, state='FOLD', width=.44),
                       dict(event='telemetry', time=1, state='FOLD', position=[1, 2, -3],
                            velocity=[.1, .2, -.3], attitude=[0, 0, 1.57], joints=[-.8]*4, gate_contacts=0)]
            (run / 'events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records)+'{"event":')
            (run / 'ground_truth.jsonl').write_text(json.dumps(dict(stamp=2, position=[2, 1, 3], quaternion=[1, 0, 0, 0]))+'\n')
            subprocess.run([sys.executable, str(runner.ROOT / 'Tools/analyze_transform_stereo.py'), str(run)],
                           env=os.environ.copy(), stdout=subprocess.DEVNULL, check=True)
            with (run / 'analysis/telemetry.csv').open() as file:
                row = next(csv.DictReader(file))
            self.assertEqual([float(row[k]) for k in ['north', 'east', 'down']], [1, 2, -3])
            with (run / 'analysis/ground_truth.csv').open() as file:
                row = next(csv.DictReader(file))
            self.assertEqual([float(row[k]) for k in ['x', 'y', 'z']], [2, 1, 3])
            self.assertFalse(json.loads((run / 'analysis/summary.json').read_text())['success'])


if __name__ == '__main__':
    unittest.main()

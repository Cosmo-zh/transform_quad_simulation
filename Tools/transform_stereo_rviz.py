"""Manage this run's isolated ROS image bridge and RViz processes."""
import os
from pathlib import Path
import shutil
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[1]


class StereoRviz:
    def __init__(self, run, env, setup, fps=5, scale=2):
        self.run = run
        self.processes = []
        self.logs = []
        self.fps, self.scale = fps, scale
        if not setup.is_file():
            raise RuntimeError(f'ROS setup not found: {setup}')
        # Source only in a child shell; paths are positional arguments, not shell code.
        result = subprocess.run(['bash', '-c', 'source "$1" >&2 && env -0', 'ros-env', str(setup)],
                                env=env, capture_output=True, timeout=15, check=True)
        self.env = dict(item.split('=', 1) for item in result.stdout.decode().split('\0') if '=' in item)
        # Give both children the exact partition used by this Gazebo instance.
        self.env['GZ_PARTITION'] = env['GZ_PARTITION']
        self.env['ROS_LOG_DIR'] = str(run / 'ros_logs')
        self.env['ROS_LOCALHOST_ONLY'] = '1'
        # OpenCV's private Qt plugin path can prevent RViz from starting.
        self.env.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
        self.env.pop('QT_QPA_FONTDIR', None)
        rviz = shutil.which('rviz2', path=self.env.get('PATH'))
        if not rviz:
            prefix = setup.parent
            candidate = prefix / 'lib/rviz2/rviz2'
            rviz = str(candidate) if candidate.is_file() else None
        if not rviz:
            raise RuntimeError('RViz2 not installed. Install ros-humble-rviz2, then retry --rviz.')
        self.rviz = rviz
        subprocess.run(['/usr/bin/python3', '-c',
                        'import rclpy; from sensor_msgs.msg import Image; import gz.transport13; import gz.msgs10.image_pb2'],
                       env=self.env, check=True, timeout=15)

    def start(self):
        config = self.run / 'stereo.rviz'
        shutil.copyfile(ROOT / 'Tools/transform_stereo.rviz', config)
        commands = [('/usr/bin/python3', str(ROOT / 'Tools/transform_stereo_ros_bridge.py'),
                     '--output', str(self.run / 'ros_images.json'), '--fps', str(self.fps),
                     '--scale', str(self.scale)),
                    (self.rviz, '-d', str(config))]
        for name, command in zip(['ros_bridge', 'rviz'], commands):
            log = (self.run / f'{name}.txt').open('w')
            self.logs.append(log)
            self.processes.append(subprocess.Popen(['nice', '-n', '10', *command], env=self.env, stdout=log,
                                                  stderr=subprocess.STDOUT, start_new_session=True))

    def check(self):
        for name, process in zip(['ROS image bridge', 'RViz2'], self.processes):
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited (code {process.returncode}); see ros_bridge.txt / rviz.txt')

    def stop(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in self.logs:
            log.close()

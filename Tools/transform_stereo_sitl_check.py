#!/usr/bin/env python3
"""Isolated stereo evidence runner. Default: static cameras, no flight.

--flight explicitly starts the companion controller and arms the 5103 SITL.
"""
import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def acquire_run_lock(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    handle = path.open('a+')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        raise RuntimeError('Another stereo run is active; wait for it to finish before starting another.')
    return handle


def finish_run(run, metadata, env, flight):
    """Generate exports after all writers have closed; retain failed runs too."""
    with (run / 'analysis.txt').open('w') as log:
        try:
            result = subprocess.run([sys.executable, str(ROOT / 'Tools/analyze_transform_stereo.py'),
                                     str(run)], env=env, stdout=log, stderr=subprocess.STDOUT, timeout=45)
            metadata['analysis_success'] = result.returncode == 0
        except (OSError, subprocess.TimeoutExpired) as exc:
            metadata.update(analysis_success=False, analysis_error=str(exc))
            log.write(f'Analysis failed; raw data retained: {exc}\n')
    metadata['finished_at'] = datetime.datetime.now().astimezone().isoformat()
    (run / 'run.json').write_text(json.dumps(metadata, indent=2)+'\n')
    manifest = []
    for path in sorted(run.rglob('*')):
        if path.is_symlink() or not path.is_file() or path.name == 'manifest.json':
            continue
        digest = hashlib.sha256()
        with path.open('rb') as reader:
            for chunk in iter(lambda: reader.read(1024*1024), b''):
                digest.update(chunk)
        manifest.append(dict(path=str(path.relative_to(run)), bytes=path.stat().st_size,
                             sha256=digest.hexdigest()))
    (run / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'Data saved: {run}', flush=True)
    if (run / 'analysis/trajectory.png').exists():
        print(f'Trajectory: {run / "analysis/trajectory.png"}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flight', action='store_true')
    parser.add_argument('--fold-check', action='store_true', help='Hover, fold to 87.5%%, reopen and land; no gate traversal')
    parser.add_argument('--gui', action='store_true', help='Open Gazebo and follow the vehicle during this run')
    parser.add_argument('--rviz', action='store_true', help='Open Gazebo and RViz2 stereo images together')
    parser.add_argument('--ros-setup', type=Path, default=Path('/opt/ros/humble/setup.bash'))
    parser.add_argument('--rviz-fps', type=float, default=5, help='Stereo preview rate (0 < fps <= 15)')
    parser.add_argument('--rviz-scale', type=int, choices=[1, 2, 4], default=2,
                        help='Preview downsample factor; original perception and recordings are unchanged')
    parser.add_argument('--output-root', type=Path, default=ROOT / 'logs/transform_stereo',
                        help='Parent of timestamped data directories (default: logs/transform_stereo)')
    parser.add_argument('--image-interval', type=float, default=1.0,
                        help='Save a stereo pair every N seconds; 0 disables the sequence')
    parser.add_argument('--duration', type=float, help='Maximum companion runtime in seconds')
    parser.add_argument('--sim-speed', type=float, help='Simulation speed factor (GUI default 0.5, headless default 1)')
    parser.add_argument('--contact-probe', action='store_true', help='Drop an unpowered model onto the gate to test contacts')
    parser.add_argument('--fixture-x', type=float, default=0, help='Static camera test position, world X')
    parser.add_argument('--gates', type=int, default=2)
    args = parser.parse_args()
    if args.fold_check:
        args.flight = True
    if args.rviz:
        args.gui = True
    if args.sim_speed is None:
        args.sim_speed = .5 if args.gui else 1.
    if not 0 < args.sim_speed <= 1:
        parser.error('--sim-speed must be between 0 and 1')
    if not 0 < args.rviz_fps <= 15:
        parser.error('--rviz-fps must be between 0 and 15')
    if args.flight and args.contact_probe:
        parser.error('--flight and --contact-probe are mutually exclusive')
    if args.gates not in (1, 2) or not 0 <= args.image_interval < float('inf'):
        parser.error('gates must be 1 or 2 and image interval must be finite and nonnegative')
    if args.duration is not None and not 0 < args.duration < float('inf'):
        parser.error('duration must be positive and finite')
    if args.gui and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        parser.error('--gui requires a desktop session (DISPLAY or WAYLAND_DISPLAY)')
    try:
        run_lock = acquire_run_lock(ROOT / 'logs/transform_stereo/.active.lock')
    except RuntimeError as exc:
        parser.error(str(exc))
    run = args.output_root.expanduser().resolve() / ('transform_stereo_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir(parents=True)
    metadata = dict(started_at=datetime.datetime.now().astimezone().isoformat(),
                    arguments={key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
                    status='running')
    (run / 'run.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print(f'Evidence: {run}', flush=True)
    build = ROOT / 'build/px4_sitl_default'
    env = os.environ.copy()
    env.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    env.update(GZ_PARTITION=run.name, HEADLESS='1',
               PX4_SIM_SPEED_FACTOR=str(args.sim_speed),
               PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION='python',
               PX4_SYS_AUTOSTART='5103', PX4_SIM_MODEL='gz_transform_quad_plus_stereo',
               PX4_GZ_MODELS=str(ROOT / 'Tools/simulation/gz/models'),
               PX4_GZ_WORLDS=str(ROOT / 'Tools/simulation/gz/worlds'),
               PX4_GZ_PLUGINS=str(build / 'src/modules/simulation/gz_plugins'),
               PX4_GZ_SERVER_CONFIG=str(ROOT / 'src/modules/simulation/gz_bridge/server.config'))
    env['GZ_SIM_RESOURCE_PATH'] = env['PX4_GZ_MODELS']+':'+env['PX4_GZ_WORLDS']
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = env['PX4_GZ_PLUGINS']
    env['GZ_SIM_SERVER_CONFIG_PATH'] = env['PX4_GZ_SERVER_CONFIG']
    (run / 'environment.json').write_text(json.dumps({key: env[key] for key in
        ['GZ_PARTITION', 'PX4_SYS_AUTOSTART', 'PX4_SIM_MODEL', 'GZ_SIM_SERVER_CONFIG_PATH']}, indent=2)+'\n')
    # Keep optional OpenCV separate from the user's Python installation.
    for dependencies in [ROOT / 'logs/transform_stereo_deps', Path('/tmp/transform-stereo-deps')]:
        if (dependencies / 'cv2').exists():
            env['PYTHONPATH'] = str(dependencies)+':'+env.get('PYTHONPATH', '')
            break
    server = companion = truth = gui = follow = None
    rviz = None
    gui_log = None
    exit_code = 1
    failure = None
    last_resource = 0.

    def record_resources():
        nonlocal last_resource
        now = time.monotonic()
        if now-last_resource < 1:
            return
        memory = dict(line.split(':', 1) for line in Path('/proc/meminfo').read_text().splitlines())
        record = dict(wall_time=time.time(), load_average=os.getloadavg(),
                      available_memory_mb=int(memory['MemAvailable'].split()[0])/1024,
                      free_swap_mb=int(memory['SwapFree'].split()[0])/1024)
        with (run / 'resources.jsonl').open('a') as file:
            file.write(json.dumps(record)+'\n')
        last_resource = now

    def launch_gui():
        nonlocal gui, gui_log
        if args.gui:
            gui_log = (run / 'gui.txt').open('w')
            config = run / 'gazebo.config'
            shutil.copyfile(ROOT / 'Tools/transform_stereo_gazebo.config', config)
            gui = subprocess.Popen(['nice', '-n', '10', 'gz', 'sim', '-g', '--gui-config', str(config)], env=env, stdout=gui_log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        if rviz:
            rviz.start()

    try:
        if args.rviz:
            from transform_stereo_rviz import StereoRviz
            rviz = StereoRviz(run, env, args.ros_setup.expanduser().resolve(), args.rviz_fps, args.rviz_scale)
        with (run / 'console.txt').open('w') as console:
            if args.flight:
                import pexpect
                server = pexpect.spawn(str(build / 'bin/px4'), ['-w', str(run), str(build / 'etc')],
                                       env=env, encoding='utf-8', timeout=90)
                server.logfile = console
                server.expect(r'pxh> ')
                launch_gui()
                for command in ['param set COM_RC_IN_MODE 4', 'param set NAV_RCL_ACT 0',
                                'param set COM_DL_LOSS_T 300',
                                'mavlink start -x -u 14656 -o 14640 -r 2000000 -m onboard -t 127.0.0.1',
                                'transform_control status']:
                    server.sendline(command)
                    server.expect_exact(command+'\r\n')
                    server.expect(r'pxh> ')
                deadline = time.monotonic()+15
                while time.monotonic() < deadline:
                    try:
                        server.read_nonblocking(65536, timeout=1)
                    except pexpect.TIMEOUT:
                        pass
                for command in ['listener sensor_combined 1', 'listener vehicle_local_position 1', 'mavlink status']:
                    server.sendline(command)
                    server.expect_exact(command+'\r\n')
                    server.expect(r'pxh> ')
                if gui:
                    if gui.poll() is not None:
                        raise RuntimeError('Gazebo GUI exited; see gui.txt')
                    follow = subprocess.Popen([sys.executable, str(ROOT / 'Tools/transform_stereo_gui.py')],
                                               env=env, stdout=gui_log, stderr=subprocess.STDOUT,
                                               start_new_session=True)
                truth = subprocess.Popen([sys.executable, str(ROOT / 'Tools/transform_stereo_truth.py'),
                                          '--output', str(run)], env=env)
            else:
                tree = ET.parse(ROOT / 'Tools/simulation/gz/worlds/transform_stereo_passage.sdf')
                tree.getroot().find('world/physics/real_time_factor').text = str(args.sim_speed)
                include = ET.SubElement(tree.getroot().find('world'), 'include')
                ET.SubElement(include, 'uri').text = 'model://transform_quad_plus_stereo'
                ET.SubElement(include, 'name').text = 'stereo_fixture'
                ET.SubElement(include, 'static').text = 'false' if args.contact_probe else 'true'
                ET.SubElement(include, 'pose').text = ('3 .5 3.4 0 0 0' if args.contact_probe
                                                       else f'{args.fixture_x} 0 1.52 0 0 0')
                tree.write(run / 'fixture.sdf', encoding='utf-8', xml_declaration=True)
                server = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', str(run / 'fixture.sdf')],
                                          env=env, stdout=console, stderr=subprocess.STDOUT, start_new_session=True)
                launch_gui()
            command = [sys.executable, str(ROOT / 'Tools/transform_stereo_passage.py'),
                       '--output', str(run), '--duration', str(args.duration or ((300 if args.flight else 45)/args.sim_speed)),
                       '--sim-speed', str(args.sim_speed),
                       '--image-interval', str(args.image_interval)]
            if args.flight:
                command += ['--execute', '--gates', str(args.gates), '--endpoint', 'udpin:127.0.0.1:14640']
            if args.fold_check:
                command += ['--fold-check']
            if args.contact_probe:
                command += ['--contact-probe']
            with (run / 'companion.txt').open('w') as log:
                companion = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                             start_new_session=True)
                while companion.poll() is None:
                    record_resources()
                    if rviz:
                        rviz.check()
                    if gui and gui.poll() is not None:
                        raise RuntimeError('Gazebo GUI closed during the run; see gui.txt')
                    if args.flight:
                        try:
                            server.read_nonblocking(65536, timeout=.5)
                        except pexpect.TIMEOUT:
                            pass
                    else:
                        time.sleep(.2)
                result = companion.returncode
            if (run / 'result.json').exists():
                summary = json.loads((run / 'result.json').read_text())
                if args.flight and not args.fold_check and summary.get('success'):
                    truth.terminate()
                    truth.wait(timeout=5)
                    records = [json.loads(line) for line in (run / 'ground_truth.jsonl').read_text().splitlines()]
                    # Independent exit check for the fixed training fixture. It
                    # does not provide gate poses/widths to the companion.
                    exits = sum(any(r['position'][0] > x+.4 for r in records) for x in [3, 7][:args.gates])
                    summary['ground_truth_exits'] = exits
                    crossings = []
                    for x, width in [(3, .6), (7, .44)][:args.gates]:
                        nearest = min(records, key=lambda r: abs(r['position'][0]-x)) if records else None
                        position = nearest['position'] if nearest else None
                        inside = bool(position and abs(position[0]-x) < .02 and abs(position[1]) < width/2
                                      and 1.25 < position[2] < 1.95)
                        crossings.append(dict(gate_x=x, position=position, center_inside=inside))
                    summary['ground_truth_crossings'] = crossings
                    summary['success'] = (exits == args.gates and summary['contact_messages'] > 0
                                          and all(c['center_inside'] for c in crossings))
                    result = 0 if summary['success'] else 1
                    (run / 'result.json').write_text(json.dumps(summary, indent=2)+'\n')
                if not args.flight and not args.contact_probe and summary.get('success'):
                    expected = .6 if args.fixture_x < 3 else .44
                    summary['expected_width'] = expected
                    summary['width_error'] = summary['median_width']-expected
                    summary['success'] = abs(summary['width_error']) < .015
                    result = 0 if summary['success'] else 1
                    (run / 'result.json').write_text(json.dumps(summary, indent=2)+'\n')
                print(json.dumps(summary, indent=2), flush=True)
            print((run / 'companion.txt').read_text()[-4000:], flush=True)
            exit_code = result
    except (Exception, KeyboardInterrupt) as exc:
        failure = str(exc) or 'Interrupted by user'
        exit_code = 130 if isinstance(exc, KeyboardInterrupt) else 1
        print(f'Run stopped: {failure}', flush=True)
    finally:
        # Keep PX4 alive while the companion handles interruption and requests landing.
        if companion and companion.poll() is None:
            companion.send_signal(signal.SIGINT)
            deadline = time.monotonic()+25/args.sim_speed
            while companion.poll() is None and time.monotonic() < deadline:
                if args.flight and server and server.isalive():
                    server.logfile = None
                    try:
                        server.read_nonblocking(65536, timeout=.2)
                    except (pexpect.TIMEOUT, pexpect.EOF):
                        pass
                else:
                    time.sleep(.1)
            if companion.poll() is None:
                os.killpg(companion.pid, signal.SIGKILL)
                companion.wait(timeout=5)
        if truth and truth.poll() is None:
            truth.terminate()
            truth.wait(timeout=5)
        if rviz:
            rviz.stop()
            stats_file = run / 'ros_images.json'
            if stats_file.exists():
                stats = json.loads(stats_file.read_text())
                metadata['rviz_images'] = stats
                metadata['rviz_stereo_connected'] = all(
                    stats.get('published', {}).get(side, 0) > 0
                    and stats.get('peak_subscribers', {}).get(side, 0) > 0
                    for side in ['left', 'right'])
            else:
                metadata['rviz_stereo_connected'] = False
        if follow:
            if follow.poll() is None:
                follow.terminate()
                try:
                    follow.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    follow.kill()
                    follow.wait(timeout=5)
            metadata['camera_follow_requested'] = follow.returncode == 0
        if gui:
            try:
                os.killpg(gui.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                gui.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(gui.pid, signal.SIGKILL)
                gui.wait(timeout=5)
        if gui_log:
            gui_log.close()
        if server:
            if args.flight:
                server.logfile = None
                if server.isalive():
                    server.sendline('shutdown')
                    try:
                        server.expect(pexpect.EOF, timeout=15)
                    except (pexpect.TIMEOUT, pexpect.EOF):
                        server.terminate(force=True)
            try:
                os.killpg(server.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            if not args.flight:
                try:
                    server.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait(timeout=5)
        result_file = run / 'result.json'
        if failure or not result_file.exists():
            previous = json.loads(result_file.read_text()) if result_file.exists() else {}
            previous.update(success=False, error=failure or 'Companion did not write a result', exit_code=exit_code)
            result_file.write_text(json.dumps(previous, indent=2)+'\n')
        metadata.update(status='finished', exit_code=exit_code,
                        success=json.loads(result_file.read_text()).get('success', False))
        finish_run(run, metadata, env, args.flight)
        run_lock.close()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())

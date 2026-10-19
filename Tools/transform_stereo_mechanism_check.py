#!/usr/bin/env python3
"""Exercise all four folding controllers with the base fixed; no PX4 or flight."""
import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    run = ROOT / 'logs/transform_stereo' / ('mechanism_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir(parents=True)
    os.environ.update(GZ_PARTITION=run.name, PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION='python',
                      GZ_SIM_RESOURCE_PATH=str(ROOT / 'Tools/simulation/gz/models'),
                      GZ_SIM_SERVER_CONFIG_PATH=str(ROOT / 'src/modules/simulation/gz_bridge/server.config'),
                      GZ_SIM_SYSTEM_PLUGIN_PATH=str(ROOT / 'build/px4_sitl_default/src/modules/simulation/gz_plugins'))
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.model_pb2 import Model
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='fold_test')
    physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '.004'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    model = ET.parse(ROOT / 'Tools/simulation/gz/models/transform_quad_plus_stereo/model.sdf').getroot().find('model')
    model.set('name', 'fold_fixture')
    ET.SubElement(model, 'pose').text = '0 0 2 0 0 0'
    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            link.remove(sensor)
    fixed = ET.SubElement(model, 'joint', name='fixed_base', type='fixed')
    ET.SubElement(fixed, 'parent').text = 'world'
    ET.SubElement(fixed, 'child').text = 'base_link'
    world.append(model)
    ET.ElementTree(sdf).write(run / 'fixture.sdf', encoding='utf-8', xml_declaration=True)
    node = Node()
    state = {}

    def receive(message):
        state.update(angles={joint.name: joint.axis1.position for joint in message.joint}, seen=time.monotonic())

    node.subscribe(Model, '/transform/joints', receive)
    topics = sorted({plugin.findtext('sub_topic') for plugin in model.findall(
        "plugin[@name='gz::sim::systems::JointPositionController']")})
    publishers = [node.advertise(f'/model/fold_fixture/{topic}', Double) for topic in topics]
    result = dict(success=False, command_topics=topics, stages=[])
    print(f'Mechanism evidence: {run}', flush=True)
    with (run / 'server.txt').open('w') as log:
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v', '3', str(run / 'fixture.sdf')],
                                  stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+25
            while time.monotonic() < deadline:
                if all(pub.has_connections() for pub in publishers) and state:
                    break
                if server.poll() is not None:
                    raise RuntimeError('Gazebo exited; see server.txt')
                time.sleep(.1)
            if not state or not all(pub.has_connections() for pub in publishers):
                raise RuntimeError(f'Missing controller subscriptions: {[pub.has_connections() for pub in publishers]}')
            for progress in [0, .3, .875, 1., 0, .875, 0]:
                target = -math.pi/3*progress
                started = time.monotonic()
                while time.monotonic()-started < 4:
                    for pub in publishers:
                        message = Double()
                        message.data = target
                        pub.publish(message)
                    time.sleep(.05)
                angles = [state['angles'][f'arm{i}_Joint'] for i in range(1, 5)]
                error = max(abs(angle-target) for angle in angles)
                result['stages'].append(dict(progress=progress, target=target, angles=angles, max_error=error))
                print(json.dumps(result['stages'][-1]), flush=True)
                if error > .025 or time.monotonic()-state['seen'] > .5:
                    raise RuntimeError('Joint tracking failed')
            result['success'] = True
        except (Exception, KeyboardInterrupt) as exc:
            result['error'] = str(exc)
        finally:
            os.killpg(server.pid, signal.SIGTERM)
            try:
                server.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(server.pid, signal.SIGKILL)
                server.wait(timeout=5)
            (run / 'result.json').write_text(json.dumps(result, indent=2)+'\n')
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

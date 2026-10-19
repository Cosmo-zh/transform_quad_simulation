#!/usr/bin/env python3
"""Generate an isolated stereo perception/traversal model and training world."""
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def generate():
    target = ROOT / 'models/transform_quad_plus_stereo'
    target.mkdir(exist_ok=True)
    tree = ET.parse(ROOT / 'models/transform_quad_plus_collision/model.sdf')
    model = tree.getroot().find('model')
    model.set('name', target.name)
    # Reduce the steady joint deflection under rotor reaction torque. Keep the
    # existing collision model unchanged; evaluate this gain only in 5103.
    for controller in model.findall("plugin[@name='gz::sim::systems::JointPositionController']"):
        controller.find('p_gain').text = '15'
        # The actual gear mechanism has one actuator. Each joint keeps its own
        # PID, but all four must consume the same servo command. Independent
        # servo_1..3 transport paths allow asymmetric/stale targets in SITL.
        controller.find('sub_topic').text = 'servo_0'
    base = model.find("link[@name='base_link']")
    for name, pose, size, color in [
            ('stereo_mount', '.05 0 .08 0 0 0', '.04 .15 .025', '.08 .08 .08 1'),
            ('left_camera', '.075 .06 .08 0 0 0', '.01 .022 .022', '.05 .2 .7 1'),
            ('right_camera', '.075 -.06 .08 0 0 0', '.01 .022 .022', '.05 .2 .7 1')]:
        visual = ET.SubElement(base, 'visual', name=name)
        ET.SubElement(visual, 'pose').text = pose
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = size
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'ambient').text = color
        ET.SubElement(material, 'diffuse').text = color
    # Ideal parallel stereo pair, 120 mm baseline, body FLU +X forward.
    # Mounted above the central body: cameras do not rotate with the arms.
    for side, y in [('left', .06), ('right', -.06)]:
        sensor = ET.SubElement(base, 'sensor', name=f'stereo_{side}', type='camera')
        ET.SubElement(sensor, 'pose').text = f'.08 {y} .08 0 0 0'
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '15'
        ET.SubElement(sensor, 'topic').text = f'/transform/stereo/{side}/image'
        camera = ET.SubElement(sensor, 'camera')
        ET.SubElement(camera, 'horizontal_fov').text = '1.2'
        image = ET.SubElement(camera, 'image')
        ET.SubElement(image, 'width').text = '960'
        ET.SubElement(image, 'height').text = '600'
        ET.SubElement(image, 'format').text = 'R8G8B8'
        clip = ET.SubElement(camera, 'clip')
        ET.SubElement(clip, 'near').text = '.05'
        ET.SubElement(clip, 'far').text = '20'
    plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system',
                           name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(plugin, 'topic').text = '/transform/joints'
    for i in range(1, 5):
        ET.SubElement(plugin, 'joint_name').text = f'arm{i}_Joint'
    for link in model.findall('link'):
        collisions = link.findall('collision')
        if collisions:
            sensor = ET.SubElement(link, 'sensor', name='passage_contact', type='contact')
            ET.SubElement(sensor, 'always_on').text = 'true'
            ET.SubElement(sensor, 'update_rate').text = '50'
            contact = ET.SubElement(sensor, 'contact')
            # gz-sim8 Contact system reads its topic inside <contact> (see its
            # installed contact_sensor.sdf example), unlike camera sensors.
            ET.SubElement(contact, 'topic').text = '/transform/contacts'
            for collision in collisions:
                ET.SubElement(contact, 'collision').text = collision.get('name')
    ET.indent(tree, space='  ')
    tree.write(target / 'model.sdf', encoding='utf-8', xml_declaration=True)
    config = ET.parse(ROOT / 'models/transform_quad_plus_collision/model.config')
    config.getroot().find('name').text = target.name
    config.getroot().find('description').text = 'Ideal stereo pair and joint/contact feedback for visual passage'
    config.write(target / 'model.config', encoding='utf-8', xml_declaration=True)

    tree = ET.parse(ROOT / 'worlds/transform_plus_passage.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'transform_stereo_passage')
    # Keep the old 5102 world intact. Training gates have more tracking margin.
    for old_name, new_name, gap in [('practice_gate_440mm', 'practice_gate_600mm', .6),
                                     ('fold_gate_360mm', 'fold_gate_440mm', .44)]:
        gate = world.find(f"model[@name='{old_name}']")
        if gate is None and old_name.startswith('practice_'):
            gate = world.find("model[@name='practice_gate_600mm']")
        if gate is None:
            raise ValueError(f'Missing source gate: {old_name}')
        gate.set('name', new_name)
        link = gate.find('link')
        for node in list(link):
            name = node.get('name', '')
            if name.startswith('pillar_'):
                sign = -1 if name.startswith('pillar_-1_') else 1
                side = (3.2 - gap) / 2
                node.find('pose').text = f'0 {sign*(gap/2+side/2)} 1.5 0 0 0'
                node.find('geometry/box/size').text = f'.25 {side} 3'
            elif name.startswith('edge_'):
                sign = -1 if name.startswith('edge_-1_') else 1
                node.find('pose').text = f'-.132 {sign*(gap/2+.008)} 1.6 0 0 0'
            elif name.startswith(('bottom_', 'top_')):
                size = node.find('geometry/box/size')
                values = size.text.split()
                values[1] = str(gap)
                size.text = ' '.join(values)
    ET.indent(tree, space='  ')
    tree.write(ROOT / 'worlds/transform_stereo_passage.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()

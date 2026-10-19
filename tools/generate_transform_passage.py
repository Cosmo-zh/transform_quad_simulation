#!/usr/bin/env python3
"""Build a plus-frame passage world and a swept-propeller collision model."""
from pathlib import Path
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
tree = ET.parse(root / 'worlds/transform_challenge_world.sdf')
world = tree.getroot().find('world')
world.set('name', 'transform_plus_passage')
for element in list(world):
    if element.tag == 'model' and element.get('name') != 'ground_plane':
        world.remove(element)
ground = world.find("model[@name='ground_plane']/link/visual")
ground.find('pose').text = '6 0 0 0 0 0'
ground.find('geometry/plane/size').text = '24 12'


def box(link, name, pose, size, color, collision=False):
    for tag in (['visual', 'collision'] if collision else ['visual']):
        node = ET.SubElement(link, tag, name=name + '_' + tag)
        ET.SubElement(node, 'pose').text = pose
        ET.SubElement(ET.SubElement(ET.SubElement(node, 'geometry'), 'box'), 'size').text = size
        if tag == 'visual':
            material = ET.SubElement(node, 'material')
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color


def model(name, pose):
    node = ET.SubElement(world, 'model', name=name)
    ET.SubElement(node, 'static').text = 'true'
    ET.SubElement(node, 'pose').text = pose
    return ET.SubElement(node, 'link', name='structure')


def gate(name, x, gap, color):
    link = model(name, f'{x} 0 0 0 0 0')
    # True collision aperture: z=1.25..1.95, centered at flight height 1.60 m.
    side_width = (3.2-gap)/2
    for sign in [-1, 1]:
        y = sign*(gap/2+side_width/2)
        box(link, f'pillar_{sign}', f'0 {y} 1.5 0 0 0',
            f'.25 {side_width} 3', color, True)
        box(link, f'edge_{sign}', f'-.132 {sign*(gap/2+.008)} 1.6 0 0 0',
            '.014 .016 .7', '0.05 0.85 0.72 1')
    box(link, 'bottom', '0 0 .625 0 0 0', f'.25 {gap} 1.25', color, True)
    box(link, 'top', '0 0 2.475 0 0 0', f'.25 {gap} 1.05', color, True)


markers = model('route_markers', '0 0 0 0 0 0')
box(markers, 'centerline', '6 0 .012 0 0 0', '12 .035 .01', '.08 .10 .12 1')
for name, x, length, color in [
        ('takeoff', 0, 1.5, '.15 .55 .25 1'),
        ('fold_wait', 5.0, 1.2, '.95 .65 .05 1'),
        ('unfold_wait', 9.0, 1.2, '.1 .45 .85 1'),
        ('landing', 12.0, 1.5, '.15 .55 .25 1')]:
    box(markers, name, f'{x} 0 .025 0 0 0', f'{length} 1.5 .012', color)
for x in range(1, 12):
    box(markers, f'distance_{x}', f'{x} 0 .035 0 0 0', '.025 .22 .01', '.9 .9 .9 1')
gate('practice_gate_440mm', 3, .44, '.28 .42 .48 1')
gate('fold_gate_360mm', 7, .36, '.38 .38 .40 1')
# Parallel 1:1 footprints make the predicted width change visible on the ground.
for name, center, positions, color in [
        ('open_envelope', (4.5, -2), [( .140007,0),(-.140007,0),(0,.140007),(0,-.140007)], '.2 .7 .3 1'),
        ('folded_envelope', (6, -2), [(.097574,-.073569),(-.097642,.073530),(.073644,.097444),(-.073497,-.097698)], '.15 .5 .85 1')]:
    link = model(name, f'{center[0]} {center[1]} .025 0 0 0')
    for i, (x, y) in enumerate(positions):
        visual = ET.SubElement(link, 'visual', name=f'prop_{i}')
        ET.SubElement(visual, 'pose').text = f'{x} {y} 0 0 0 0'
        cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
        ET.SubElement(cylinder, 'radius').text = '.06355'
        ET.SubElement(cylinder, 'length').text = '.004'
        material = ET.SubElement(visual, 'material')
        ET.SubElement(material, 'diffuse').text = color
        ET.SubElement(material, 'ambient').text = color
ET.indent(tree, space='  ')
tree.write(root / 'worlds/transform_plus_passage.sdf', encoding='utf-8', xml_declaration=True)

target = root / 'models/transform_quad_plus_collision'
target.mkdir(exist_ok=True)
tree = ET.parse(root / 'models/transform_quad_plus/model.sdf')
vehicle = tree.getroot().find('model')
vehicle.set('name', target.name)
# Align actual base_link/IMU forward with world +X at zero spawn yaw.
vehicle.find("link[@name='base_link']/pose").text = '0 0 0 0 0 0'
for i in range(1, 5):
    link = vehicle.find(f"link[@name='fan{i}_Link']")
    collision = ET.SubElement(link, 'collision', name='swept_propeller_collision')
    ET.SubElement(collision, 'pose').text = '0 0 -.003 0 0 0'
    cylinder = ET.SubElement(ET.SubElement(collision, 'geometry'), 'cylinder')
    ET.SubElement(cylinder, 'radius').text = '.06355'
    ET.SubElement(cylinder, 'length').text = '.006'
ET.indent(tree, space='  ')
tree.write(target / 'model.sdf', encoding='utf-8', xml_declaration=True)
config = ET.parse(root / 'models/transform_quad_plus/model.config')
config.getroot().find('name').text = target.name
config.getroot().find('description').text = 'Plus frame with swept propeller collision disks for passage tests'
config.write(target / 'model.config', encoding='utf-8', xml_declaration=True)

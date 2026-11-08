"""Build the standalone Gazebo world from the project's Harmonic physics template."""
import math
from pathlib import Path
import random
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE.parents[1] / 'gazebo_sim/worlds/dronehacks.sdf'


def box(link, name, size, shade=None, pose=None):
    item = ET.SubElement(link, 'collision' if shade is None else 'visual', name=name)
    if pose:
        ET.SubElement(item, 'pose').text = pose
    geometry = ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
    ET.SubElement(geometry, 'size').text = size
    if shade is not None:
        material = ET.SubElement(item, 'material')
        for key in ('ambient', 'diffuse'):
            ET.SubElement(material, key).text = f'{shade} {shade} {shade} 1'


def build(traversal=False):
    tree = ET.parse(TEMPLATE)
    world = tree.find('world')
    world.remove(world.find('include'))
    world.remove(world.find('gui'))
    world.set('name', 'funnel')
    world.find('scene/background').text = '0.95 0.95 0.95 1'
    rng = random.Random(60)
    angle = math.radians(30)
    length, thickness, height = 4.0, 0.10, 1.73
    for side, name in ((1, 'left'), (-1, 'right')):
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        # The downstream, inner corner is exactly (4, +/-0.5) in ENU.
        x = 4 - length/2 * math.cos(angle) + thickness/2 * math.sin(angle)
        y = side * (0.5 + length/2 * math.sin(angle) + thickness/2 * math.cos(angle))
        ET.SubElement(model, 'pose').text = f'{x} {y} {height/2} 0 0 {-side*angle}'
        link = ET.SubElement(model, 'link', name='panel')
        size = f'{length} {thickness} {height}'
        box(link, 'solid', size)
        box(link, 'surface', size, 0.09)
        # Sparse matte patches provide image corners for stereo tracking.
        for i in range(110):
            u = rng.uniform(-1.93, 1.93)
            z = rng.uniform(-0.80, 0.80)
            w, h = rng.uniform(0.03, 0.13), rng.uniform(0.03, 0.12)
            box(link, f'mark_{i}', f'{w:.4f} 0.001 {h:.4f}', rng.choice([0.01, 0.38]),
                f'{u:.4f} {-side*0.0501} {z:.4f} 0 0 0')
    if traversal:
        ground = world.find("model[@name='ground']/link")
        for i in range(440):
            x, y = rng.uniform(1.6, 12.0), rng.uniform(-3.5, 3.5)
            w, h = rng.uniform(.045, .17), rng.uniform(.05, .18)
            box(ground, f'floor_mark_{i}', f'{w:.4f} {h:.4f} .001',
                rng.choice([.025, .12, .76]), f'{x:.4f} {y:.4f} .0006 0 0 {rng.uniform(-3,3):.4f}')
    ET.indent(tree)
    tree.write(HERE / ('funnel_traversal.sdf' if traversal else 'funnel.sdf'), encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    import argparse
    args = argparse.ArgumentParser()
    args.add_argument('--traversal', action='store_true')
    build(args.parse_args().traversal)

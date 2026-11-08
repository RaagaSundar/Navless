"""Gazebo-only, seeded spawn and periodic crosswind fixture. SI units, ENU."""
import argparse
import json
from pathlib import Path
import random
import secrets
from xml.etree import ElementTree as ET

HERE = Path(__file__).resolve().parent


def assets(seed):
    rng = random.Random(seed)
    x = round(rng.uniform(-.15, .15), 4)
    y = round(rng.choice((-1, 1))*rng.uniform(.20,.34), 4)
    wind_y = rng.choice((-1, 1))*1.0
    period = round(rng.uniform(4., 7.), 3)
    world = ET.parse(HERE/'funnel_traversal.sdf')
    w = world.find('world')
    ET.SubElement(ET.SubElement(w, 'wind'), 'linear_velocity').text = f'0 {wind_y} 0'
    plugin = ET.SubElement(w, 'plugin', filename='gz-sim-wind-effects-system',
                           name='gz::sim::systems::WindEffects')
    magnitude = ET.SubElement(ET.SubElement(plugin, 'horizontal'), 'magnitude')
    ET.SubElement(magnitude, 'time_for_rise').text = '1.0'
    sine = ET.SubElement(magnitude,'sin')
    ET.SubElement(sine,'amplitude_percent').text = '0.65'
    ET.SubElement(sine,'period').text = str(period)
    ET.SubElement(plugin,'force_approximation_scaling_factor').text = '0.18'
    quad = ET.parse(HERE/'dh_quad_stereo.sdf')
    body = quad.find("model/link[@name='base_link']")
    ET.SubElement(body,'enable_wind').text = 'true'
    settings = dict(seed=seed, spawn_enu_m=[x,y,.075],yaw_deg=-6,
                    wind_enu_m_s=[0,wind_y,0],gust_amplitude_m_s=.65,period_sim_s=period,
                    wind_response_per_s=.18,wind_link='base_link')
    spawn = ('sdf_filename: "/funnel/simulated-quad.sdf", name: "dh_quad_0", '
             f'pose: {{position:{{x:{x},y:{y},z:0.075}},orientation:{{z:-0.052335956,w:0.998629535}}}}')
    return {'funnel.sdf': ET.tostring(world.getroot(),encoding='utf-8'),
            'simulated-quad.sdf': ET.tostring(quad.getroot(),encoding='utf-8'),
            'spawn-request.txt': spawn.encode(),
            'wind-fixture.json': json.dumps(settings,indent=2).encode()}


def verify(run):
    if not (run/'wind-fixture.json').exists():
        if (run/'spawn-request.txt').exists() or (run/'funnel.sdf').read_bytes() != (HERE/'funnel_traversal.sdf').read_bytes():
            raise ValueError('unrecognized traversal fixture')
        return
    seed = json.loads((run/'wind-fixture.json').read_text())['seed']
    for name,data in assets(seed).items():
        if (run/name).read_bytes() != data:
            raise ValueError(f'fixture mismatch in {name}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('operation',choices=['create','verify'])
    parser.add_argument('run',type=Path)
    parser.add_argument('--seed',type=int)
    args = parser.parse_args()
    if args.operation == 'verify':
        verify(args.run)
    else:
        seed = secrets.randbits(32) if args.seed is None else args.seed
        for name,data in assets(seed).items():
            with (args.run/name).open('xb') as f:
                f.write(data)

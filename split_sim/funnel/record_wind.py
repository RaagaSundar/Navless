"""Independent Gazebo Harmonic recorder of resolved wind and body-link state."""
import json
import signal
import time
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.serialized_map_pb2 import SerializedStepMap
from gz.transport13 import Node


def component_id(name):
    # Gazebo component registration uses FNV-1a, stored in a signed protobuf key.
    value = 14695981039346656037
    for byte in ('gz_sim_components.'+name).encode():
        value = ((value ^ byte)*1099511628211) % (1 << 64)
    return value if value < (1 << 63) else value-(1 << 64)


def record():
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM,stop)
    node = Node()
    name, parent, velocity, wind, mode = map(component_id,
                    ('Name','ParentEntity','WorldLinearVelocity','Wind','WindMode'))
    deadline = time.monotonic()+600
    while running and time.monotonic()<deadline:
        ok,msg = node.request('/world/funnel/state',Empty(),Empty,SerializedStepMap,2000)
        if ok:
            entities = msg.state.entities
            model = next((i for i,e in entities.items() if name in e.components
                          and e.components[name].component==b'dh_quad_0'),None)
            field = next((e.components for e in entities.values() if wind in e.components),None)
            body = next((e.components for e in entities.values() if name in e.components
                        and e.components[name].component==b'base_link' and parent in e.components
                        and int(e.components[parent].component)==model),None)
            if field and body and velocity in field and velocity in body:
                stamp=msg.stats.sim_time
                print(json.dumps(dict(sim_s=stamp.sec+stamp.nsec*1e-9,
                    wind_m_s=list(map(float,field[velocity].component.split())),
                    body_velocity_m_s=list(map(float,body[velocity].component.split())),
                    wind_enabled=mode in body and body[mode].component==b'1')),flush=True)
        time.sleep(.30)


if __name__ == '__main__':
    record()

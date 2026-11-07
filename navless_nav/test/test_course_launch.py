"""navless_course.launch.py passes every speed setting it declares on to navless.launch.py.

The drone's competitor window starts this launch with no arguments, so its defaults are what flies.
A declared but unforwarded argument would be silently dropped. Parsed with ast: no ROS needed.
"""
import ast
import os

LAUNCH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch', 'navless_course.launch.py')
SPEED = ('cruise_speed_mps', 'max_lead_m', 'max_lat_accel_mps2')


def _declared_and_forwarded():
    tree = ast.parse(open(LAUNCH, encoding='utf-8').read())
    args = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                and any(getattr(t, 'id', '') == 'ARGUMENTS' for t in n.targets))
    declared = {e.elts[0].value: e.elts[1].value for e in args.value.elts}
    forwarded = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.DictComp):
            for gen in node.generators:
                if isinstance(gen.iter, ast.Tuple):
                    forwarded |= {e.value for e in gen.iter.elts if isinstance(e, ast.Constant)}
    return declared, forwarded


def test_speed_settings_declared_and_forwarded():
    declared, forwarded = _declared_and_forwarded()
    for name in SPEED:
        assert name in declared, name
        assert name in forwarded, name


def test_lead_lets_the_drone_reach_cruise_speed():
    # PX4 (MPC_XY_P 0.95) flies a position-only setpoint at about 0.95 x the distance to it, and the
    # carrot never gets more than max_lead_m ahead: a shorter lead caps the speed below cruise.
    declared, _ = _declared_and_forwarded()
    assert float(declared['max_lead_m']) * 0.95 >= float(declared['cruise_speed_mps'])


def test_everything_forwarded_is_declared():
    declared, forwarded = _declared_and_forwarded()
    assert forwarded <= set(declared), forwarded - set(declared)

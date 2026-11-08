"""navless_course.launch.py passes its speed settings on, and the slow variant stays at the base defaults.

The drone's competitor window starts these launches with no arguments, so their defaults are what
flies. A declared but unforwarded argument would be silently dropped. Parsed with ast: no ROS needed.
"""
import ast
import os

LAUNCH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch')
COURSE = os.path.join(LAUNCH_DIR, 'navless_course.launch.py')
SLOW = os.path.join(LAUNCH_DIR, 'navless_course_slow.launch.py')
GATE_B = os.path.join(LAUNCH_DIR, 'navless_course_gate_b.launch.py')
BASE = os.path.join(LAUNCH_DIR, 'navless.launch.py')
SPEED = ('cruise_speed_mps', 'max_lead_m', 'max_lat_accel_mps2', 'max_turn_sagitta_m')


def _tree(path):
    return ast.parse(open(path, encoding='utf-8').read())


def _assigned(tree, name):
    return next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
                and any(getattr(t, 'id', '') == name for t in n.targets))


def _arguments(path):
    """name -> default from a launch file's ARGUMENTS list of (name, default, help)."""
    return {e.elts[0].value: e.elts[1].value for e in _assigned(_tree(path), 'ARGUMENTS').elts}


def _forwarded(path):
    """Names in the tuple a launch file's dict comprehension forwards to the included launch."""
    out = set()
    for node in ast.walk(_tree(path)):
        if isinstance(node, ast.DictComp):
            for gen in node.generators:
                if isinstance(gen.iter, ast.Tuple):
                    out |= {e.value for e in gen.iter.elts if isinstance(e, ast.Constant)}
    return out


def _dict_values(path, name):
    d = _assigned(_tree(path), name)
    return {k.value: v.value for k, v in zip(d.keys, d.values)}


def _slow_values():
    return _dict_values(SLOW, 'SLOW')


def _gate_b_values():
    return _dict_values(GATE_B, 'GATE_B')


def test_speed_settings_declared_and_forwarded():
    declared, forwarded = _arguments(COURSE), _forwarded(COURSE)
    for name in SPEED:
        assert name in declared, name
        assert name in forwarded, name


def test_lead_lets_the_drone_reach_cruise_speed():
    # PX4 (MPC_XY_P 0.95) flies a position-only setpoint at about 0.95 x the distance to it, and the
    # carrot never gets more than max_lead_m ahead: a shorter lead caps the speed below cruise.
    for args in (_arguments(COURSE), _slow_values(), _gate_b_values()):
        assert float(args['max_lead_m']) * 0.95 >= float(args['cruise_speed_mps'])


def test_everything_forwarded_is_declared():
    declared, forwarded = _arguments(COURSE), _forwarded(COURSE)
    assert forwarded <= set(declared), forwarded - set(declared)


def test_slow_launch_uses_the_base_defaults():
    base, slow = _arguments(BASE), _slow_values()
    for name in ('cruise_speed_mps', 'max_lead_m', 'max_lat_accel_mps2'):
        assert slow[name] == base[name], name


def test_slow_launch_overrides_only_what_the_course_launch_forwards():
    assert set(_slow_values()) <= set(_arguments(COURSE)) & _forwarded(COURSE)
    assert "'navless_course.launch.py'" in open(SLOW, encoding='utf-8').read()


def test_gate_b_launch_overrides_only_what_the_course_launch_forwards():
    assert set(_gate_b_values()) <= set(_arguments(COURSE)) & _forwarded(COURSE)
    assert "'navless_course.launch.py'" in open(GATE_B, encoding='utf-8').read()


def test_gate_b_launch_flies_the_base_speeds_with_the_turn_cap():
    # navless_course.launch.py's faster speeds were tuned for the window route only.
    base, gate_b = _arguments(BASE), _gate_b_values()
    for name in ('cruise_speed_mps', 'max_lead_m', 'max_lat_accel_mps2'):
        assert gate_b[name] == base[name], name
    assert float(gate_b['max_turn_sagitta_m']) == 0.08
    assert gate_b['route_file'] != _arguments(COURSE)['route_file']

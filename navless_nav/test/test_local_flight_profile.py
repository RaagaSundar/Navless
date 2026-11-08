"""Exercise the slow wrapper's argument resolution without ROS or a flight process."""
from pathlib import Path
import runpy
import sys
import types


SLOW = Path(__file__).resolve().parents[1] / 'launch' / 'navless_course_slow.launch.py'


def resolve_slow_launch(monkeypatch, overrides):
    class Declare:
        def __init__(self, name, default_value):
            self.name, self.default_value = name, default_value

    class Configuration:
        def __init__(self, name):
            self.name = name

        def perform(self, context):
            return context[self.name]

    class Include:
        def __init__(self, source, launch_arguments):
            self.source, self.arguments = source, dict(launch_arguments)

    modules = {
        'launch': {'LaunchDescription': list},
        'launch.actions': {'DeclareLaunchArgument': Declare, 'IncludeLaunchDescription': Include},
        'launch.substitutions': {'LaunchConfiguration': Configuration},
        'launch.launch_description_sources': {'PythonLaunchDescriptionSource': lambda path: path},
        'ament_index_python': {},
        'ament_index_python.packages': {'get_package_share_directory': lambda name: '/packages/' + name},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)

    actions = runpy.run_path(str(SLOW))['generate_launch_description']()
    context = dict(overrides)
    for action in actions:
        if isinstance(action, Declare):
            context.setdefault(action.name, action.default_value)
        else:
            assert isinstance(action, Include)
            assert action.source.endswith('navless_course.launch.py')
            context.update({name: value.perform(context) if isinstance(value, Configuration) else value
                            for name, value in action.arguments.items()})
    return context


def test_slow_wrapper_keeps_existing_defaults(monkeypatch):
    context = resolve_slow_launch(monkeypatch, {})
    assert context == {'cruise_speed_mps': '0.5', 'max_lead_m': '0.6', 'max_lat_accel_mps2': '0.25'}


def test_requested_speed_and_lead_reach_course_launch(monkeypatch):
    context = resolve_slow_launch(monkeypatch, {'cruise_speed_mps': '0.3', 'max_lead_m': '0.4'})
    assert context['cruise_speed_mps'] == '0.3'
    assert context['max_lead_m'] == '0.4'
    assert context['max_lat_accel_mps2'] == '0.25'


def test_slow_wrapper_retains_other_explicit_course_arguments(monkeypatch):
    overrides = {'route_source': 'file', 'localizer': 'false', 'wall_correction': 'false',
                 'replan_on_obstacles': 'false', 'heading_mode': 'hold',
                 'drift_seed_body_y_m': '0.0', 'use_sim_time': 'true',
                 'max_lat_accel_mps2': '0.12'}
    context = resolve_slow_launch(monkeypatch, overrides)
    assert all(context[name] == value for name, value in overrides.items())
    assert context['cruise_speed_mps'] == '0.5'
    assert context['max_lead_m'] == '0.6'

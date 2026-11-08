"""The staged qualification launch's contract, resolved without ROS or a flight process."""
from pathlib import Path
import re
import runpy
import sys
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_local_flight_profile import resolve_slow_launch    # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
QUAL = ROOT / 'navless_nav' / 'launch' / 'navless_course_qualification.launch.py'
COURSE = ROOT / 'navless_nav' / 'launch' / 'navless_course.launch.py'


def resolve(monkeypatch, overrides):
    """(arguments the qualification launch hands the slow launch, what that forwards on)."""
    class Declare:
        def __init__(self, name, default_value, description=''):
            self.name, self.default_value = name, default_value

    class Opaque:
        def __init__(self, function):
            self.function = function

    class Include:
        def __init__(self, source, launch_arguments):
            self.source, self.arguments = source, dict(launch_arguments)

    modules = {
        'launch': {'LaunchDescription': list},
        'launch.actions': {'DeclareLaunchArgument': Declare, 'IncludeLaunchDescription': Include,
                           'OpaqueFunction': Opaque},
        'launch.launch_description_sources': {'PythonLaunchDescriptionSource': lambda path: path},
        'ament_index_python': {},
        'ament_index_python.packages': {'get_package_share_directory': lambda name: '/packages/' + name},
    }
    for name, attributes in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    actions = runpy.run_path(str(QUAL))['generate_launch_description']()
    configurations = dict(overrides)
    for action in actions:
        if isinstance(action, Declare):
            configurations.setdefault(action.name, action.default_value)
    (opaque,) = [a for a in actions if isinstance(a, Opaque)]
    (include,) = opaque.function(types.SimpleNamespace(launch_configurations=configurations))
    assert include.source.endswith('navless_course_slow.launch.py')
    # an included launch sees the parent's configurations, its launch_arguments taking precedence
    return include.arguments, resolve_slow_launch(monkeypatch, {**configurations, **include.arguments})


def test_shadow_is_the_default_and_moves_nothing(monkeypatch):
    args, course = resolve(monkeypatch, {})
    assert course['wall_correction'] == 'false' and course['replan_on_obstacles'] == 'false'
    assert course['localizer'] == 'true' and course['stereo_cloud'] == 'true'
    # the candidate starting settings, forwarded through the slow wrapper (not its 0.5/0.6/0.25)
    assert (course['cruise_speed_mps'], course['max_lead_m'], course['max_lat_accel_mps2']) == \
        ('0.3', '0.4', '0.12')
    assert course['max_turn_sagitta_m'] == '0.015' and course['heading_mode'] == 'hold'
    assert course['drift_seed_body_y_m'] == '0.0'            # explicit, not the course's -0.15
    assert course['odometry_topic'] == course['validated_odom_topic'] == '/mapping/validated_odometry'


@pytest.mark.parametrize('phase,wall,replan', [('correction', 'true', 'false'),
                                               ('mapping', 'true', 'true')])
def test_later_phases_enable_feedback_in_order(monkeypatch, phase, wall, replan):
    _, course = resolve(monkeypatch, {'phase': phase})
    assert (course['wall_correction'], course['replan_on_obstacles']) == (wall, replan)


@pytest.mark.parametrize('overrides', [{'phase': 'flight'}, {'phase': ''},
                                       {'wall_correction': 'true'},
                                       {'phase': 'shadow', 'replan_on_obstacles': 'True'},
                                       {'phase': 'correction', 'replan_on_obstacles': 'true'},
                                       {'localizer': 'false'}, {'stereo_cloud': 'false'}])
def test_bad_phase_or_conflicting_feedback_switch_is_refused(monkeypatch, overrides):
    with pytest.raises(ValueError):
        resolve(monkeypatch, overrides)


def test_matching_switches_and_other_settings_pass_through(monkeypatch):
    _, course = resolve(monkeypatch, {'phase': 'correction', 'wall_correction': 'true',
                                      'cruise_speed_mps': '0.25', 'route_source': 'file'})
    assert course['wall_correction'] == 'true' and course['cruise_speed_mps'] == '0.25'
    assert course['route_source'] == 'file'


def test_course_launch_gives_the_localizer_its_own_validated_topic():
    text = COURSE.read_text(encoding='utf-8')
    assert "('validated_odom_topic', '/mapping/validated_odometry'" in text
    assert "'validated_odom_topic:=', LaunchConfiguration('validated_odom_topic')" in text
    assert "'validated_odom_topic:=', LaunchConfiguration('odometry_topic')" not in text


def test_no_new_px4_command_paths():
    """Nothing here publishes to /fmu/in, arms or changes modes: flight_interface owns that."""
    files = [ROOT / 'navless_nav' / 'navless_nav' / 'course_mission.py', QUAL,
             ROOT / 'hardware_slam' / 'odom_gate.py', ROOT / 'tools' / 'navigation_readiness.py',
             ROOT / 'navless_nav' / 'navless_nav' / 'navigation_integrity.py',
             ROOT / 'navless_nav' / 'navless_nav' / 'correction_contract.py']
    for path in files:
        text = path.read_text(encoding='utf-8')
        assert not re.search(r"create_publisher\([^)]*fmu", text), path
        assert not re.search(r"VehicleCommand|OffboardControlMode|create_client|call_async", text), path

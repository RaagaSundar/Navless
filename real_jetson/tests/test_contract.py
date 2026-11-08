from pathlib import Path
import json
import math
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'jetson'), str(ROOT/'mac')]
from dashboard import Feed, project
from handover import Handover
from plan import plan
from plan_source.route_lib.route import create_live_course_planner


def test_route_matches_planner_and_inflated_map():
    route = json.loads((ROOT/'jetson/window-route.json').read_text())
    assert json.loads(json.dumps(plan())) == route
    points = route['waypoints_enu']
    assert points[0][:2] == route['launch_origin_enu']
    assert math.dist(points[-1][:2], [-2.042, -.792]) < .02
    _, grid, _ = create_live_course_planner()
    for a, b in zip(points, points[1:]):
        steps = max(1, math.ceil(math.dist(a, b)/.025))
        for i in range(steps + 1):
            point = [v+(w-v)*i/steps for v,w in zip(a,b)]
            assert not grid.is_occupied(point)
            assert max(map(abs,point[:2])) <= 3.048-.25


@pytest.mark.parametrize('loss', ['sensor expired','pilot takeover','disarm'])
def test_handover_never_resumes_after_loss(loss):
    g = Handover()
    def sample(t, fault='', armed=False, off=False, fi=False):
        return g.update(t, fault, True, armed, armed, off, fi)
    assert sample(0) == 'WAIT'
    assert sample(2.1) == 'READY'
    assert sample(2.2,armed=True,off=True) == 'READY'
    assert sample(2.3,armed=True,off=True,fi=True) == 'ACTIVE'
    assert sample(2.4,loss if loss=='sensor expired' else '', loss!='disarm', loss!='pilot takeover', True) == 'STOP'
    assert sample(2.5,armed=True,off=True,fi=True) == 'STOP'


def test_dashboard_marks_missing_fc_unknown_and_refuses_stale_or_sim():
    sample = {'schema_version':1,'env':{'ros_domain_id':19},'written_at':100.,'seq':9,
              'topics':{k:{'age_s':0} for k in ('slam_status','slam_odometry','vehicle_status','skybeta_state')},
              'fc':{'link':False,'armed':False},'vio':{'vo_state_name':'tracking','odom_position_enu':[1,2,3]},
              'skybeta_state':{'armed':False,'landed':True,'offboard_active':False}}
    current=project(sample,100.1,.01)
    assert current['pose'] == [1,2,3]
    assert all(current[k] is None for k in ('armed','offboard','landed','mode','px4'))
    with pytest.raises(ValueError): project(sample,115,0)
    sample['env']['ros_domain_id']=219
    with pytest.raises(ValueError): project(sample,100,0)
    feed=Feed()
    feed.set(current)
    feed.received-=10
    assert feed.get()['status']=='offline'


def test_historical_control_topics_match_candidate():
    # Source receipt from the installed hover controller that corresponds to the archived flight recordings.
    import ast
    for name in ['pose','twist']:
        assert f"'/skybeta/setpoint/{name}'" in (ROOT/'jetson/interface_monitor.py').read_text()
    tree=ast.parse((ROOT/'jetson/navless_nav/course_mission.py').read_text())
    writers=[n.args[1].id for n in ast.walk(tree) if isinstance(n,ast.Call)
             and isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher']
    assert sorted(writers)==['POSE_TOPIC','TWIST_TOPIC']

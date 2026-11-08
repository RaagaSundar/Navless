"""Independent grade for one archived windy traversal, with run-local comparison."""
import hashlib
import json
import math
from pathlib import Path
from statistics import median
import sys
from review_traversal import review

BASE = Path(__file__).resolve().parent
sys.path.insert(0,str(BASE.parent))
from run_comparison import write


def save(run, index):
    baseline = BASE/'runs/20261108T034242Z'
    cfg=json.loads((run/'wind-fixture.json').read_text())
    settings=json.loads((baseline/'settings.json').read_text())
    settings['disturbance']=cfg
    (run/'settings.json').write_text(json.dumps(settings,indent=2))
    (run/'renderer-source.json').write_text(json.dumps({key:hashlib.sha256((run/name).read_bytes()).hexdigest()
        for key,name in [('world_sha256','funnel.sdf'),('spawn_sdf_sha256','simulated-quad.sdf')]},indent=2))
    result=review(run)
    rows=json.loads((run/'flight/track.json').read_text())['samples']
    air=[s for s in rows if s['armed'] and not s['landed'] and s['status_age']<=1.5]
    launch=[s for s in rows if not s['armed'] and s['landed'] and (not air or s['t']<air[0]['t'])]
    wind=[json.loads(line) for line in (run/'wind.jsonl').read_text().splitlines() if line.strip()]
    gust=[s for s in wind if air and air[0]['t']<=s['sim_s']<=air[-1]['t']]
    speeds=[math.hypot(*s['wind_m_s']) for s in gust]
    gaps=[b['sim_s']-a['sim_s'] for a,b in zip(gust,gust[1:])]
    finished=run/'flight/finished'
    operator_stopped=(run/'flight/operator-stop-status.log').is_file()
    if not finished.is_file() and not operator_stopped:
        raise ValueError('missing pilot exit record')
    result.update(operator_stopped=operator_stopped,index=index,seed=cfg['seed'],configured_offset_m=cfg['spawn_enu_m'][1],
        measured_spawn_xy_m=[median(s[k] for s in launch) for k in ('x','y')] if launch else None,
        gust_period_s=cfg['period_sim_s'],wind_mean_y_m_s=cfg['wind_enu_m_s'][1],
        airborne_wind_samples=len(gust),wind_min_m_s=min(speeds,default=None),
        wind_max_m_s=max(speeds,default=None),wind_max_gap_sim_s=max(gaps,default=None),
        windy_link_verified=bool(gust) and all(s['wind_enabled'] for s in gust),
        mission_exit=int(finished.read_text()) if finished.is_file() else None)
    (run/'flight/result.json').write_text(json.dumps(result,indent=2))
    annotations=dict(notes=['See flight/result.json for the independent grade and wind.jsonl for sampled Gazebo state.'],
        confounds=['Launch position and wind change together. Gust phase at takeoff depends on startup timing. Gazebo wind response is a simplified force model.'],
        changes=[dict(field='initial position and wind',before='centered spawn and calm air',after=cfg,
                      evidence=['wind-fixture.json','simulated-quad.sdf','spawn-request.txt','wind.jsonl'])])
    write(run,name=f'Windy funnel {index:02d} · seed {cfg["seed"]}',kind='flight attempted',baseline=baseline,annotations=annotations)
    return result


if __name__=='__main__':
    result=save(Path(sys.argv[1]),int(sys.argv[2]))
    print(json.dumps({k:result[k] for k in ('index','run_id','seed','crossing_verified','lateral_error_m','wall_contacts','final_landed_disarmed','windy_link_verified')}))

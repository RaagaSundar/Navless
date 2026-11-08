"""Print the window route from the frozen planner and its as-built occupancy grid."""
import json
from pathlib import Path
from plan_source.route_lib.route import plan_dynamic_course_route


def plan():
    for name in ("course_config.json", "prior_course_map.json"):
        if not (Path(__file__).parent / "planner/config" / name).is_file():
            raise FileNotFoundError(name)
    points, launch, kind = plan_dynamic_course_route(branch="window")
    return dict(route_type=kind, frame="course_enu", launch_origin_enu=launch,
                waypoints_enu=points, planner_commit="7a7883250c6e783c357a14d56ed9626d7300ba58")


if __name__ == "__main__":
    print(json.dumps(plan(), indent=2, allow_nan=False))

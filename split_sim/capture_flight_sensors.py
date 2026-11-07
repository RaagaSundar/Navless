"""Record original split Gazebo stereo and body IMU for isolated estimator replay."""
import argparse
import json
import os
from pathlib import Path
import sqlite3
import threading
import time
import zlib

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.imu_pb2 import IMU

BODY = "dh_quad_0::base_link::imu_sensor"
IMU_TOPIC = "/world/dronehacks/model/dh_quad_0/link/base_link/sensor/imu_sensor/imu"

def capture(path, stop_file, max_wall):
    if os.environ.get("GZ_PARTITION") != "drone19_split" or path.exists():
        raise ValueError("requires split partition and a new file")
    db = sqlite3.connect(path, check_same_thread=False)
    db.execute("CREATE TABLE samples(stamp INTEGER, kind TEXT, frame TEXT, data BLOB, PRIMARY KEY(stamp,kind))")
    lock = threading.Lock()
    done = threading.Event()
    last_info = {}
    first = None
    def record(kind,m):
        nonlocal first
        stamp = m.header.stamp.sec*10**9 + m.header.stamp.nsec
        frames = [v for d in m.header.data if d.key=="frame_id" for v in d.value]
        with lock:
            if done.is_set():
                return
            if first is None:
                first=stamp
            if stamp < first-1_000_000_000:
                done.set()
                return
            if len(frames)!=1:
                raise ValueError("ambiguous source frame")
            if kind=="imu":
                if frames!=[BODY] or m.entity_name!=BODY:
                    raise ValueError("unexpected inertial sensor")
                # Discard simulator orientation. cuVSLAM receives accelerometer and gyro only.
                payload = json.dumps([getattr(v,k) for v in (m.angular_velocity,m.linear_acceleration) for k in "xyz"])
            elif kind.endswith("info"):
                payload=json.dumps(dict(width=m.width,height=m.height,k=list(m.intrinsics.k),p=list(m.projection.p),
                                        d=list(m.distortion.k),r=list(m.rectification_matrix),model=m.distortion.model))
                if last_info.get(kind)==payload:
                    return
                last_info[kind]=payload
            else:
                if m.pixel_format_type!=1 or m.width!=640 or m.height!=400 or m.step!=640:
                    raise ValueError("unexpected mono source")
                payload=zlib.compress(m.data,1)
            db.execute("INSERT INTO samples VALUES (?,?,?,?)",(stamp,kind,frames[0],payload))
    node=Node()
    node.subscribe(IMU,IMU_TOPIC,lambda m:record("imu",m))
    for eye in ("left","right"):
        node.subscribe(Image,f"/oak/{eye}/image_raw",lambda m,e=eye:record(e,m))
        node.subscribe(CameraInfo,f"/oak/{eye}/camera_info",lambda m,e=eye:record(e+"_info",m))
    print("subscribed to original calibrated stereo and body IMU", flush=True)
    deadline=time.monotonic()+max_wall
    while not done.wait(.2) and not stop_file.exists() and time.monotonic()<deadline:
        pass
    done.set()
    with lock:
        db.commit()
        print(list(db.execute("SELECT kind,frame,count(*),min(stamp),max(stamp) FROM samples GROUP BY kind,frame")),flush=True)
        db.close()
    # gz-transport's pybind callback destructors can run without the GIL at
    # interpreter shutdown. All ingress is disabled and the archive is closed.
    os._exit(0)

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path",type=Path)
    parser.add_argument("--stop-file",type=Path,required=True)
    parser.add_argument("--max-wall",type=float,default=1200)
    args=parser.parse_args()
    if args.stop_file.exists():
        raise SystemExit('refusing an old sensor-capture stop marker')
    capture(args.path,args.stop_file,args.max_wall)

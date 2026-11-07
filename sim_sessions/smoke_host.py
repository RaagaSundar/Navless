#!/usr/bin/env python3
"""Administrator-invoked CPU smoke only. No install, keys, GPU, ROS nodes or flights."""
import argparse
import contextlib
import json
import multiprocessing
from pathlib import Path
import subprocess
import tempfile
import time
import uuid
from gateway import lease
from layout import Slot

PROBE = """import json,os,sys
from pathlib import Path
from shutil import which
data=json.loads(input())
up=[p.parent.name for p in Path('/sys/class/net').glob('*/flags') if int(p.read_text(),16)&1]
assert set(up)=={'lo'}
assert not any(Path(p).exists() for p in ('/dev/nvhost-gpu','/dev/nvmap','/dev/ttyACM0','/dev/ttyTHS0','/dev/bus/usb','/var/run/docker.sock'))
print(json.dumps(dict(data=data,domain=os.environ['ROS_DOMAIN_ID'],uid=os.getuid(),up=up,
    has_env=Path('/usr/local/bin/skybeta_env.sh').is_file(),has_ros=Path('/opt/ros/humble').is_dir(),
    cgroup_memory=[p.read_text().strip() for p in (Path('/sys/fs/cgroup/memory.max'),Path('/sys/fs/cgroup/memory/memory.limit_in_bytes')) if p.is_file()],
    has_inner_sshd=bool(which('sshd') or Path('/usr/sbin/sshd').is_file()))))
"""


def worker(raw,i,image,tag,events,first_go):
    slot = Slot(i)
    with lease(Path(raw),slot,timeout=16):
        events.put((time.monotonic_ns(),'admitted',i))
        if i==14 and not first_go.wait(5):
            raise TimeoutError('second client did not queue')
        name = f'navless-cpu-smoke-{tag}-{i}'
        cmd=['docker','run','--rm','-i','--pull=never','--name',name,'--runtime=runc','--network=none',
             '--user=65534:65534','--ipc=private','--shm-size=4m','--cpus=.15','--memory=64m',
             '--memory-swap=64m','--pids-limit=16','--cap-drop=ALL','--read-only',
             '--security-opt=no-new-privileges','-e','NVIDIA_VISIBLE_DEVICES=void',
             '-e',f'ROS_DOMAIN_ID={slot.domain}','--entrypoint','python3',image,'-c',PROBE]
        result=subprocess.run(cmd, input=json.dumps({'tag':tag,'i':i})+'\n',text=True,
                              capture_output=True, timeout=12)
        if result.returncode:
            raise RuntimeError(result.stderr[-900:])
        report=json.loads(result.stdout)
        if report['data']!={'tag':tag,'i':i} or report['domain']!=str(slot.domain) or report['uid']!=65534:
            raise RuntimeError('session mismatch')
        events.put((time.monotonic_ns(),'finished',i,report))


def main(image):
    available=int(next(line.split()[1] for line in Path('/proc/meminfo').read_text().splitlines()
                       if line.startswith('MemAvailable:')))
    if available<512*1024:
        raise RuntimeError('less than 512 MiB available; defer the smoke')
    image_id=subprocess.check_output(['docker','image','inspect','-f','{{.Id}}',image],text=True).strip()
    ctx=multiprocessing.get_context('fork')
    events=ctx.Queue()
    first_go=ctx.Event()
    tag=uuid.uuid4().hex[:12]
    timeline=[]
    with tempfile.TemporaryDirectory(prefix='navless-cpu-smoke-') as raw, contextlib.ExitStack() as cleanup:
        for i in (14,15):
            process=ctx.Process(target=worker,args=(raw,i,image_id,tag,events,first_go))
            process.start()
            cleanup.callback(process.join,2)
            cleanup.callback(process.terminate)
            for _ in range(100):
                if len(list(Path(raw).glob('ticket-*')))==i-13:
                    break
                time.sleep(.02)
            else:
                raise RuntimeError('client did not queue')
        first_go.set()
        try:
            for _ in range(4):
                timeline.append(events.get(timeout=18))
            timeline=[v[1:] for v in sorted(timeline)]
            if [(v[0],v[1]) for v in timeline]!=[('admitted',14),('finished',14),('admitted',15),('finished',15)]:
                raise RuntimeError('unexpected scheduling order')
        finally:
            pending=[]
            for i in (14,15):
                name=f'navless-cpu-smoke-{tag}-{i}'
                if subprocess.run(['docker','inspect',name],capture_output=True).returncode==0:
                    subprocess.run(['docker','rm','-f',name],check=True,stdout=subprocess.DEVNULL)
                    pending.append(name)
            if pending:
                raise RuntimeError('worker cleanup required manual removal: '+', '.join(pending))
        print(json.dumps(dict(image_id=image_id,timeline=timeline,containers_remaining=0),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image',default='skybeta:0.3.3')
    args=p.parse_args()
    main(args.image)

#!/usr/bin/env python3
"""Run a single foreground session; all client assets live in a private namespace."""
import argparse
import contextlib
import os
from pathlib import Path
import re
import signal
import socketserver
import subprocess
import tempfile
import threading
import uuid
from layout import Slot
from bundle import encode

HERE = Path(__file__).resolve().parent


def replace(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError('unsupported split script; expected once: ' + old)
    return text.replace(old,new)


def prepare(split, out, slot):
    for name in ('run_laptop.sh','start_px4.sh','sensor_link.py','make_assets.py','range_sensor.py'):
        s = (split / name).read_text()
        if name in ('run_laptop.sh','start_px4.sh'):
            s = replace(s, 'GZ_PARTITION=drone19_split', f'GZ_PARTITION={slot.name}')
        if name == 'run_laptop.sh':
            s = replace(s, 'setsid gz sim -s -r', 'setsid gz sim -s --headless-rendering -r')
            s = replace(s,'setsid gz sim -g > /root/logs/gui.log 2>&1 & pids+=("$!")',
                        '# This session uses only the headless renderer.')
        if name == 'start_px4.sh':
            s = replace(s, '== 219', f'== {slot.domain}')
            s = replace(s, 'PX4_PARAM_UXRCE_DDS_DOM_ID=219', f'PX4_PARAM_UXRCE_DDS_DOM_ID={slot.domain}')
            s = replace(s, 'PX4_UXRCE_DDS_PORT=8889', f'PX4_UXRCE_DDS_PORT={slot.port(3)}')
        if name == 'make_assets.py' and 'Path(__file__).resolve().parents[1]' in s:
            s = replace(s, 'Path(__file__).resolve().parents[1]', "Path('/workspace/Navless')")
        if name == 'sensor_link.py':
            s = replace(s, "os.environ.get('ROS_DOMAIN_ID') != '219'",
                        f"os.environ.get('ROS_DOMAIN_ID') != '{slot.domain}'")
        (out / name).write_text(s)


class Relay(socketserver.ThreadingMixIn,socketserver.UnixStreamServer):
    daemon_threads = True


@contextlib.contextmanager
def endpoint(path, name, port):
    class Handler(socketserver.BaseRequestHandler):
        def handle(self):
            subprocess.run(['docker','exec','-i',name,'python3','/session/stdio_tcp.py',str(port)],
                           stdin=self.request, stdout=self.request, check=False)
    with Relay(str(path), Handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            yield
        finally:
            server.shutdown()
            thread.join()


def main(args):
    slot = Slot(args.slot)
    repo, state = args.repo.resolve(), args.state.resolve()
    if not re.fullmatch('[a-zA-Z0-9][a-zA-Z0-9_.-]{0,62}',args.gateway):
        raise ValueError('gateway must be a simple SSH config alias')
    if state.name != slot.name:
        raise ValueError('--state must be a dedicated directory named '+slot.name)
    if not (state / 'PX4-Autopilot/build/px4_sitl_default/bin/px4').is_file():
        raise ValueError('need a private prepared split PX4 build in --state')
    payload = encode(args.program)
    import fcntl
    with (state / '.session.lock').open('a') as file, tempfile.TemporaryDirectory(prefix='ns-',dir='/tmp') as raw:
        fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        temp = Path(raw)
        split = temp / 'split'
        split.mkdir()
        prepare(repo/'split_sim',split,slot)
        name = slot.name + '-laptop'
        nonce = uuid.uuid4().hex
        inner = ['ssh', '-F','/dev/null','-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
                 '-o','UserKnownHostsFile='+str(args.known_hosts.resolve()),
                 '-o','HostKeyAlias='+slot.name,'-o','IdentitiesOnly=yes','-i',str(args.key.resolve()),
                 '-o','ServerAliveInterval=10','-o','ServerAliveCountMax=2','-o','ExitOnForwardFailure=yes',
                 '-o','ProxyCommand=ssh -T -o BatchMode=yes -o StrictHostKeyChecking=yes -o ClearAllForwardings=yes -o ControlMaster=no -o ControlPath=none -o ServerAliveInterval=10 -o ServerAliveCountMax=2 ' + args.gateway,
                 '-R',f'127.0.0.1:{slot.port(1)}:{temp}/camera',
                 '-R',f'127.0.0.1:{slot.port(2)}:{temp}/xrce','participant@'+slot.name]
        cmd = ['docker','run','-d','--init','--pull=never','--name',name,'--network','none',
               '--label','navless.client='+nonce,'--ipc','private','--shm-size','1g',
               '--security-opt','no-new-privileges','-e',f'ROS_DOMAIN_ID={slot.domain}',
               '-e','ROS_LOCALHOST_ONLY=1','-e',f'GZ_PARTITION={slot.name}','-e','JETSON_IP=127.0.0.1',
               '-e','SPLIT_INERTIAL=0','-e','SPLIT_APPEARANCE=original',
               '-v',f'{state}:/root','-v',f'{repo}:/workspace/Navless:ro',
               '-v',f'{split}:/split:ro','-v',f'{HERE}:/session:ro','--entrypoint','bash',
               args.image,'-c','sleep infinity']
        cid = subprocess.check_output(cmd,text=True).strip()
        if not re.fullmatch(r"[a-f0-9]{64}", cid):
            raise RuntimeError("docker did not return a container ID")
        try:
            with contextlib.ExitStack() as stack:
                stack.enter_context(endpoint(temp/'camera',cid,7447))
                stack.enter_context(endpoint(temp/'xrce',cid,slot.port(2)))
                # Wait for the renderer to create the model before starting PX4.
                script = '''
set -eo pipefail
source /opt/ros/jazzy/setup.bash
python3 /session/datagrams.py laptop "$1" "$2" &
bash /split/run_laptop.sh &
for i in {1..120}; do
  if gz topic -l | grep -F '/model/dh_quad_0/' >/dev/null; then break; fi
  if [[ "$i" == 120 ]]; then exit 2; fi
  sleep 1
done
bash /split/start_px4.sh &
wait -n
exit 1
'''
                sim = subprocess.Popen(['docker','exec',cid,'bash','-c',script,'session',
                                        str(slot.port(2)),str(slot.port(3))])
                with tempfile.TemporaryFile() as program:
                    program.write(payload)
                    program.seek(0)
                    ssh = subprocess.Popen(inner,stdin=program)
                    try:
                        import time
                        while ssh.poll() is None and sim.poll() is None:
                            time.sleep(.25)
                        if sim.poll() is not None:
                            raise RuntimeError('laptop simulator exited; inspect logs in --state')
                        return ssh.returncode
                    finally:
                        if ssh.poll() is None:
                            ssh.terminate()
                            ssh.wait(timeout=10)
        finally:
            subprocess.run(['docker','rm','-f',cid],check=False)
            if 'sim' in locals():
                sim.wait(timeout=10)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slot',required=True,type=int)
    parser.add_argument('--gateway',required=True)
    for name in ('repo','state','key','known-hosts','program'):
        parser.add_argument('--'+name,required=True,type=Path)
    parser.add_argument('--image',default='dronehacks-local:jazzy-harmonic')
    for sig in (signal.SIGHUP,signal.SIGTERM):
        signal.signal(sig,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    raise SystemExit(main(parser.parse_args()))

#!/usr/bin/env python3
"""PID 1 child: fixed-key inner sshd reachable ONLY through stdin/stdout."""
import os
import pwd
import signal
import subprocess
import sys
from pathlib import Path
from layout import Slot


def main(slot):
    if any(p.parent.name != 'lo' and int(p.read_text(),16) & 1
           for p in Path('/sys/class/net').glob('*/flags')):
        raise RuntimeError('network must be none')
    for p in ('/dev/ttyACM0','/dev/ttyTHS0','/dev/bus/usb','/var/run/docker.sock'):
        if os.path.exists(p):
            raise RuntimeError('unexpected physical/control device')
    uid = pwd.getpwnam('participant')
    os.chown('/work', uid.pw_uid, uid.pw_gid)
    os.makedirs('/run/sshd', exist_ok=True)
    # Private, session-local copy; retain the exact single allowed domain check.
    source = Path('/opt/source/sensor_link.py').read_text()
    guard = "os.environ.get('ROS_DOMAIN_ID') != '219'"
    if source.count(guard) != 1:
        raise RuntimeError('unsupported split sensor guard')
    source = source.replace('/tmp/split_link_metrics.jsonl','/run/stack/split_link_metrics.jsonl')
    Path('/run/stack').mkdir(mode=0o755)
    Path('/run/stack/sensor_link.py').write_text(source.replace(guard, f"os.environ.get('ROS_DOMAIN_ID') != '{slot.domain}'"))
    opts = f"""
HostKey /slot/host_key
AuthorizedKeysFile /slot/client.pub
UsePAM no
PermitRootLogin no
PasswordAuthentication no
KbdInteractiveAuthentication no
AuthenticationMethods publickey
AllowUsers participant
PermitTTY no
AllowAgentForwarding no
X11Forwarding no
PermitTunnel no
AllowTcpForwarding remote
AllowStreamLocalForwarding no
GatewayPorts no
PermitListen 127.0.0.1:{slot.port(1)} 127.0.0.1:{slot.port(2)}
MaxSessions 1
ClientAliveInterval 10
ClientAliveCountMax 2
ForceCommand /bin/bash /opt/session/participant.sh {slot.domain}
"""
    Path('/run/session-sshd').write_text(opts)
    with open('/run/stack/session-stack.log','wb') as log:
        stack = subprocess.Popen(['bash','/opt/session/stack.sh',str(slot.domain),
                                  str(slot.port(1)),str(slot.port(2)),str(slot.port(3))],
                                 stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
        ssh = subprocess.Popen(['/usr/sbin/sshd','-i','-e','-f','/run/session-sshd'])
        import time
        try:
            while ssh.poll() is None and stack.poll() is None:
                time.sleep(.2)
            if stack.poll() is not None:
                print(Path(log.name).read_text(errors='replace')[-3000:],file=sys.stderr)
                return 1
            return ssh.returncode
        finally:
            ssh.terminate()
            if stack.poll() is None:
                os.killpg(stack.pid, signal.SIGTERM)


if __name__ == '__main__':
    sys.exit(main(Slot(int(sys.argv[1]))))

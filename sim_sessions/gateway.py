#!/usr/bin/env python3
"""Administrator-owned forced SSH command. Does not interpret SSH_ORIGINAL_COMMAND."""
import contextlib
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))  # trusted installer path under python -I
from layout import Slot

BASE = Path('/opt/navless-sessions')
STATE = Path('/var/lib/navless-sessions')
DOCKER = '/usr/bin/docker'


def lock(path, blocking=True):
    handle = path.open('a')
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
    except BaseException:
        handle.close()
        raise
    return handle


@contextlib.contextmanager
def lease(state, slot, timeout=900, gone=lambda: False):
    """Each waiter holds a flock on its ticket; a dead process cannot block FIFO."""
    try:
        owner = lock(state / (slot.name + '.lock'), False)
    except BlockingIOError:
        raise RuntimeError('this operator already has a waiting or active session')
    with owner:
        with lock(state / 'sequence.lock'):
            seq = state / 'sequence'
            number = int(seq.read_text() if seq.exists() else '0') + 1
            with tempfile.NamedTemporaryFile(mode='w', dir=state, delete=False) as f:
                f.write(str(number))
            os.replace(f.name, seq)
            ticket = state / f'ticket-{number:020d}'
            token = lock(ticket)
        try:
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and not gone():
                with lock(state / 'sequence.lock'):
                    for other in sorted(state.glob('ticket-*')):
                        if other == ticket:
                            # Keep this ticket locked until container cleanup finishes.
                            break
                        try:
                            with lock(other, False):
                                other.unlink(missing_ok=True)
                        except BlockingIOError:
                            break
                    if other == ticket:
                        break
                time.sleep(.15)
            else:
                raise TimeoutError('session cancelled or queue expired')
            yield
        finally:
            with lock(state / 'sequence.lock'):
                ticket.unlink(missing_ok=True)
                token.close()


def run(slot):
    os.umask(0o077)
    if not (BASE / 'ENABLED').is_file():
        raise RuntimeError('administrator has not enabled the simulator window')
    poll = select.poll()
    poll.register(0, select.POLLHUP | select.POLLERR)
    print(f'{slot.name}: waiting for GPU admission', file=sys.stderr, flush=True)
    with lease(STATE, slot, gone=lambda: any(mask & (select.POLLHUP | select.POLLERR)
                                             for _, mask in poll.poll(0))):
        if not (BASE / 'ENABLED').is_file():
            raise RuntimeError('simulator window closed')
        names = subprocess.check_output([DOCKER, 'ps', '--format', '{{.Names}}'], text=True).split()
        if any(n.startswith('navless-gpu-') or n in ('drone19-split-sim','drone19-gazebo-bench')
               for n in names):
            raise RuntimeError('another workload or orphan exists; ask the administrator')
        name = 'navless-gpu-' + slot.name + '-' + uuid.uuid4().hex
        volume = slot.name + '-work'
        # Never auto-create an unbounded volume on aircraft storage.
        vol = json.loads(subprocess.check_output([DOCKER,'volume','inspect',volume]))[0]
        if (vol['Driver'] != 'local' or vol.get('Options') !=
              {'type':'tmpfs','device':'tmpfs','o':'size=128m,mode=0755'}):
            raise RuntimeError('administrator must provision the bounded session volume')
        config = BASE / 'slots' / slot.name
        image = (BASE / 'image-id').read_text().strip()
        import re
        if not re.fullmatch(r'sha256:[0-9a-f]{64}', image):
            raise RuntimeError('image-id must be an inspected local image ID')
        cmd = [DOCKER, 'run', '--rm', '-i', '--pull=never', '--init', '--name', name,
               '--label', 'navless.session=' + slot.name, '--network', 'none', '--ipc', 'private',
               '--pids-limit', '256', '--memory', '3500m', '--memory-swap', '3500m',
               '--cpus', '3', '--shm-size', '384m', '--runtime', 'nvidia',
               '--security-opt', 'no-new-privileges', '--cap-drop', 'ALL',
               '--read-only','--tmpfs','/run:size=8m','--tmpfs','/tmp:size=200m,mode=1777',
               '--mount', f'type=volume,src={volume},dst=/work',
               '--mount', f'type=bind,src={config},dst=/slot,readonly',
               '-e', 'NVIDIA_VISIBLE_DEVICES=all','-e','NVIDIA_DRIVER_CAPABILITIES=compute,utility']
        for cap in ('SETUID','SETGID','SYS_CHROOT','CHOWN','DAC_OVERRIDE','KILL','AUDIT_WRITE','FOWNER'):
            cmd += ['--cap-add', cap]
        print(f'{slot.name}: admitted; limited to 30 minutes',file=sys.stderr,flush=True)
        try:
            subprocess.run(cmd + [image, str(slot.number)], check=True, timeout=1800)
        finally:
            # Only this invocation's UUID; never stop another team's container.
            subprocess.run([DOCKER, 'rm', '-f', name], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=25)


if __name__ == '__main__':
    for sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, lambda *_: sys.exit(1))
    run(Slot(int(sys.argv[1])))

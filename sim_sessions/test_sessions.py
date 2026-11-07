import contextlib
import fcntl
import getpass
import multiprocessing as mp
import os
from pathlib import Path
import shutil
import socket
import struct
import subprocess as sp
import sys
import tempfile
import time
import unittest
import uuid
from client import endpoint, HERE
from gateway import lease
from layout import Slot
from bundle import encode, unpack


def waiter(state, i, output, hold):
    with lease(Path(state), Slot(i), timeout=10):
        output.put(i)
        hold.wait(10)


class SessionTests(unittest.TestCase):
    def test_workspace_boundary(self):
        with tempfile.TemporaryDirectory() as raw:
            source, dest = Path(raw)/'source', Path(raw)/'out'
            source.mkdir()
            dest.mkdir()
            (source/'lib').mkdir()
            (source/'main.py').write_text('from lib import x')
            (source/'lib/x.py').write_text('value=42')
            unpack(encode(source),dest)
            self.assertEqual((dest/'lib/x.py').read_text(),'value=42')
            with self.assertRaises(ValueError):
                unpack(b'{"../escape":"eA==","main.py":""}',dest)
            (source/'link').symlink_to('/etc/hosts')
            with self.assertRaises(ValueError):
                encode(source)

    def test_queue_duplicate_death_and_order(self):
        ctx = mp.get_context('spawn')
        output = ctx.Queue()
        with tempfile.TemporaryDirectory() as raw, contextlib.ExitStack() as cleanup:
            state = Path(raw)
            processes = []
            for i in range(1,5):
                hold = ctx.Event()
                proc = ctx.Process(target=waiter,args=(raw,i,output,hold))
                proc.start()
                cleanup.callback(proc.join,3)
                cleanup.callback(proc.terminate)
                processes.append((proc,hold))
                for _ in range(100):
                    if len(list(state.glob('ticket-*'))) == i:
                        break
                    time.sleep(.025)
                else:
                    self.fail('missing ticket')
            self.assertEqual(output.get(timeout=2),1)
            with self.assertRaisesRegex(RuntimeError,'already'):
                with lease(state,Slot(2)):
                    pass
            processes[1][0].kill()  # Abandoned queue ticket.
            processes[1][0].join()
            processes[0][0].kill()  # Host gateway died while holding the slot.
            processes[0][0].join()
            self.assertEqual(output.get(timeout=3),3)
            processes[2][1].set()
            self.assertEqual(output.get(timeout=3),4)

    @unittest.skipUnless(os.environ.get('TEST_DOCKER_SSH')=='1','explicit lightweight local Docker and SSH test')
    def test_two_network_none_clients(self):
        image=os.environ.get('TEST_IMAGE','ros:humble-ros-base-jammy')
        with tempfile.TemporaryDirectory(dir='/tmp',prefix='ns-test-') as raw, contextlib.ExitStack() as all_jobs:
            sessions = []
            for i in (14,15):
                job = all_jobs.enter_context(contextlib.ExitStack())
                slot = Slot(i)
                name = slot.name+'-test-'+uuid.uuid4().hex[:8]
                root=Path(raw)/str(i)
                root.mkdir()
                run = lambda *a: sp.run(a,check=True,stdout=sp.DEVNULL)
                for key in ('server','client'):
                    run('ssh-keygen','-q','-t','ed25519','-N','','-f',str(root/key))
                (root/'known').write_text(slot.name+' '+(root/'server.pub').read_text())
                (root/'sshd').write_text(f"""HostKey {root}/server
AuthorizedKeysFile {root}/client.pub
StrictModes no
UsePAM no
AuthenticationMethods publickey
AllowTcpForwarding remote
GatewayPorts no
PermitListen 127.0.0.1:{slot.port(1)} 127.0.0.1:{slot.port(2)}
ForceCommand /bin/cat
""")
                run('docker','run','--rm','-d','--network','none','--name',name,
                    '-e',f'ROS_DOMAIN_ID={slot.domain}','-v',f'{HERE}:/session:ro','--entrypoint',
                    'python3',image,'/session/datagrams.py','laptop',str(slot.port(2)),str(slot.port(3)))
                job.callback(sp.run,['docker','rm','-f',name],stdout=sp.DEVNULL,stderr=sp.DEVNULL)
                self.assertEqual(sp.check_output(['docker','inspect','-f','{{.HostConfig.NetworkMode}}',name],text=True).strip(),'none')
                run('docker','exec',name,'python3','-c',"from pathlib import Path; assert not any(p.parent.name!='lo' and int(p.read_text(),16)&1 for p in Path('/sys/class/net').glob('*/flags'))")
                run('docker','exec','-d',name,'python3','-c',
                    "import os,socket; s=socket.socket(); s.bind(('127.0.0.1',7447)); s.listen(); "
                    "c,a=s.accept(); c.sendall(('camera:'+os.environ['ROS_DOMAIN_ID']).encode()); c.close()")
                job.enter_context(endpoint(root/'camera',name,7447))
                job.enter_context(endpoint(root/'xrce',name,slot.port(2)))
                ssh = sp.Popen(['ssh','-F','/dev/null','-T','-o','BatchMode=yes','-o','IdentitiesOnly=yes',
                    '-o','ExitOnForwardFailure=yes','-o','StrictHostKeyChecking=yes','-o',
                    'UserKnownHostsFile='+str(root/'known'),'-o',f'ProxyCommand=/usr/sbin/sshd -i -f {root}/sshd',
                    '-i',str(root/'client'),'-R',f'127.0.0.1:{slot.port(1)}:{root}/camera',
                    '-R',f'127.0.0.1:{slot.port(2)}:{root}/xrce',
                    getpass.getuser()+'@'+slot.name],stdin=sp.PIPE,stdout=sp.PIPE)
                def stop(proc):
                    proc.terminate()
                    proc.wait(timeout=5)
                    for stream in (proc.stdin,proc.stdout):
                        if stream: stream.close()
                job.callback(stop,ssh)
                ssh.stdin.write(b'ready\n')
                ssh.stdin.flush()
                self.assertEqual(ssh.stdout.readline(),b'ready\n')
                echo=job.enter_context(socket.socket(socket.AF_INET,socket.SOCK_DGRAM))
                echo.bind(('127.0.0.1',slot.port(3)))
                echo.settimeout(5)
                relay=sp.Popen([sys.executable,str(HERE/'datagrams.py'),'jetson',str(slot.port(2)),str(slot.port(3))])
                job.callback(stop,relay)
                sessions.append((job,slot,name,echo,ssh))
            def roundtrip(session, msg):
                _, slot, name, echo, _ = session
                code = """import os,socket,sys
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.bind(('127.0.0.1',31518))
s.settimeout(.4)
for _ in range(14):
 s.sendto(sys.argv[2].encode(),('127.0.0.1',int(sys.argv[1])))
 try:
  print(os.environ['ROS_DOMAIN_ID'],s.recv(4096).decode(),flush=True)
  break
 except socket.timeout: pass
else: raise SystemExit(3)
"""
                p=sp.Popen(['docker','exec',name,'python3','-c',code,str(slot.port(3)),msg],stdout=sp.PIPE,text=True)
                try:
                    data, peer = echo.recvfrom(4096)
                    self.assertEqual(data,msg.encode())
                    echo.sendto(b'reply:'+data,peer)
                    self.assertEqual(p.communicate(timeout=6)[0].strip(),f'{slot.domain} reply:{msg}')
                    self.assertEqual(p.returncode,0)
                finally:
                    if p.poll() is None:
                        p.kill()
                        p.wait()
            for _,slot,_,_,_ in sessions:
                with socket.create_connection(('127.0.0.1',slot.port(1)),timeout=3) as c:
                    self.assertEqual(c.makefile('rb').read(),f'camera:{slot.domain}'.encode())
            roundtrip(sessions[0],'alpha')
            roundtrip(sessions[1],'bravo')
            sessions[0][4].kill()  # Abrupt authenticated tunnel loss.
            sessions[0][4].wait(timeout=5)
            sessions[0][0].close()
            self.assertNotEqual(sp.run(['docker','inspect',sessions[0][2]],capture_output=True).returncode,0)
            with self.assertRaises(OSError):
                socket.create_connection(('127.0.0.1',sessions[0][1].port(2)),timeout=.5)
            roundtrip(sessions[1],'bravo-after-alpha-exit')


if __name__ == '__main__':
    unittest.main()

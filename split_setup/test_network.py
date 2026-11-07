import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from split_setup.install_network_fix import install, OLD_IP, OLD_SSH

HELPER = Path(__file__).with_name('split_network.sh').resolve()


class NetworkTests(unittest.TestCase):
    def run_helper(self, expression, **settings):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            log = root / 'ssh.log'
            fake = root / 'ssh'
            fake.write_text('''#!/usr/bin/env python3
import json, os, sys
with open(os.environ['SSH_TEST_LOG'], 'a') as f:
    f.write(json.dumps(sys.argv[1:]) + '\\n')
if '-O' in sys.argv:
    sys.exit(1)
if '-M' in sys.argv:
    sys.exit(0 if sys.argv[-1] == 'operator@192.0.2.8' else 1)
if os.environ.get('SSH_FAIL'):
    sys.exit(255)
print(os.environ.get('SSH_CONNECTION_TEST', '192.0.2.1 50123 169.254.55.2 22'))
''')
            fake.chmod(0o755)
            env = dict(os.environ)
            for key in list(env):
                if key.startswith('JETSON_'):
                    del env[key]
            env.update(PATH=str(root) + os.pathsep + env['PATH'], SSH_TEST_LOG=str(log))
            env.update(settings)
            result = subprocess.run(['bash', '-c', 'set -euo pipefail; source "$1"; ' + expression,
                                     'test', str(HELPER)], env=env, capture_output=True, text=True)
            return result, log.read_text() if log.exists() else ''

    def test_ssh_server_address_beats_wrong_mdns(self):
        result, log = self.run_helper('split_select_xrce_ip')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '169.254.55.2')
        self.assertIn('HostKeyAlias=skybeta-19.local', log)
        self.assertIn('StrictHostKeyChecking=yes', log)
        self.assertIn('BatchMode=yes', log)

    def test_explicit_ip_wins_without_ssh(self):
        result, log = self.run_helper('split_select_xrce_ip', JETSON_IP='192.0.2.8', SSH_FAIL='1')
        self.assertEqual(result.stdout.strip(), '192.0.2.8')
        self.assertEqual(result.returncode, 0)
        self.assertEqual(log, '')

    def test_bad_addresses_fail(self):
        for value in ('::1', 'not-an-ip', '0.0.0.0', '127.0.0.1', '224.1.1.1'):
            result, _ = self.run_helper('split_select_xrce_ip', JETSON_IP=value)
            self.assertNotEqual(result.returncode, 0, value)

    def test_missing_malformed_or_failed_connection_fails(self):
        for options in ({'SSH_CONNECTION_TEST': ''}, {'SSH_CONNECTION_TEST': 'too few'},
                        {'SSH_CONNECTION_TEST': '192.0.2.1 1 ::1 22'}, {'SSH_FAIL': '1'}):
            result, _ = self.run_helper('split_select_xrce_ip', **options)
            self.assertNotEqual(result.returncode, 0)

    def test_explicit_ssh_target_and_alias(self):
        result, log = self.run_helper('split_select_xrce_ip',
                                     JETSON_SSH_TARGET='operator@192.0.2.8',
                                     JETSON_HOST_KEY_ALIAS='reviewed-host')
        self.assertEqual(result.returncode, 0)
        self.assertIn('operator@192.0.2.8', log)
        self.assertIn('HostKeyAlias=reviewed-host', log)

    def test_fallback_is_operator_supplied_and_strict(self):
        result, log = self.run_helper('split_ensure_ssh; printf "%s" "$TARGET"',
                                     JETSON_SSH_FALLBACK='operator@192.0.2.8')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, 'operator@192.0.2.8')
        self.assertIn('StrictHostKeyChecking=yes', log)
        result, log = self.run_helper('split_ensure_ssh', JETSON_SSH_TARGET='operator@192.0.2.9',
                                     JETSON_SSH_FALLBACK='operator@192.0.2.8')
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn('operator@192.0.2.8', log)

    def test_installer_validates_both_before_writing_and_is_idempotent(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            start = root / 'start.sh'; px4 = root / 'start_px4_from_mac.sh'
            start.write_text(OLD_SSH); px4.write_text('unknown version')
            with self.assertRaises(ValueError):
                install(root)
            self.assertEqual(start.read_text(), OLD_SSH)
            self.assertFalse((root / 'split_network.sh').exists())
            px4.write_text(OLD_IP)
            install(root)
            before = {p.name: p.read_text() for p in root.iterdir()}
            install(root)
            self.assertEqual(before, {p.name: p.read_text() for p in root.iterdir()})
            self.assertIn('split_ensure_ssh', start.read_text())
            self.assertIn('split_select_xrce_ip', px4.read_text())


if __name__ == '__main__':
    unittest.main()

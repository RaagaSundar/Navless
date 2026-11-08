import ast,hashlib,importlib.util,unittest
from pathlib import Path
HERE=Path(__file__).resolve().parent
ORIGINAL=next(p for p in [HERE.parent/'viewer/map_server.py',HERE/'original_map_server_local_source.py',HERE/'map_server_local_source.py'] if p.exists())
spec=importlib.util.spec_from_file_location('probe',HERE/'passive_pair_probe.py');probe=importlib.util.module_from_spec(spec);spec.loader.exec_module(probe)
class PassiveProbeTest(unittest.TestCase):
    def test_source_sha_mismatch_fails_before_ROS(self):
        with self.assertRaises(ValueError):probe.load_pair(HERE/'roundoff.py','wrong',ORIGINAL,'wrong')
    def test_pinned_original_and_candidate_load_without_ROS_or_GPU(self):
        sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest();original=ORIGINAL
        candidate,valid=probe.load_pair(HERE/'roundoff.py',sha(HERE/'roundoff.py'),original,sha(original))
        self.assertTrue(callable(valid));self.assertTrue(callable(candidate.canonicalize_message))
    def test_no_application_publisher_parameter_reset_or_command_calls(self):
        tree=ast.parse((HERE/'passive_pair_probe.py').read_text());forbidden={'create_publisher','publish','set_parameters','set_parameters_atomically','system','Popen','run','check_call'}
        calls=[node.func.attr for node in ast.walk(tree) if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute)]
        self.assertFalse(forbidden.intersection(calls));self.assertIn('create_subscription',calls);self.assertIn('sched_setaffinity',calls)
if __name__=='__main__':unittest.main()

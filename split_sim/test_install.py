"""Fresh-layout, missing dependency and frozen flight manifest validation."""
import importlib.util,shutil,tempfile,unittest
from pathlib import Path
spec=importlib.util.spec_from_file_location('splitdoctor',Path(__file__).with_name('doctor.py'));d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
class CleanInstall(unittest.TestCase):
 def test_fresh_space_path_layout(self):
  with tempfile.TemporaryDirectory(prefix='split clean install ') as name:
   target=Path(name)/'Navless fresh checkout'
   shutil.copytree(d.ROOT,target,ignore=shutil.ignore_patterns('.git','state','__pycache__','*.pyc'))
   report=d.check_source(target);self.assertEqual(report['frozen_runtime_members'],38)
 def test_missing_prerequisite_rejected(self):
  with tempfile.TemporaryDirectory() as name:
   with self.assertRaisesRegex(ValueError,'missing tracked'):d.check_source(Path(name))
if __name__=='__main__':unittest.main()

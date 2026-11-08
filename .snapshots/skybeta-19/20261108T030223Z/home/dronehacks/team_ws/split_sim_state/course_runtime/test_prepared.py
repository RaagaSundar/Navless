import tempfile
import unittest
from pathlib import Path

from startup import prepared_approved


class ActivationTests(unittest.TestCase):
    def test_binds_run_and_deployed_sources(self):
        with tempfile.TemporaryDirectory() as tmp:
            activation = Path(tmp)/'approval'
            self.assertFalse(prepared_approved(activation,'run1','manifest'))
            for text in ('run0 manifest','run1 previous','run1','run1 manifest unexpected'):
                activation.write_text(text)
                self.assertFalse(prepared_approved(activation,'run1','manifest'))
            activation.write_text('run1 manifest\n')
            self.assertTrue(prepared_approved(activation,'run1','manifest'))
            activation.unlink()
            self.assertFalse(prepared_approved(activation,'run1','manifest'))

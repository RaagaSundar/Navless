"""Pair a wall correction with its acquisition-time confidence, never receipt time.

cloud_localize publishes /team/localize/status (JSON) and, only for a qualified cloud,
/team/localize/odom_correction stamped with that cloud's capture time. The status carries
`correction_source_stamp_ns`, the exact integer stamp of the cloud the current correction
came from; a correction is used only when a status with that stamp says it was a fully
observed tracking match. Either message may arrive first.
"""
import math


class CorrectionContract:
    def __init__(self, max_age_s=0.5, future_tolerance_s=0.005):
        self.max_age = max_age_s        # capture -> acceptance, source time
        self.future = future_tolerance_s
        self.samples = {}
        self.epoch = None
        self.last_ns = None             # newest accepted correction stamp
        self.fault = None
        self.current_valid = False      # the NEWEST status still vouches for a correction
        self.reason = 'no matched localization status'

    def status(self, data):
        if data.get('source_epoch_fault'):
            self.fault = str(data['source_epoch_fault'])
        self.current_valid = data.get('correction_valid') is True and self.fault is None
        try:
            stamp = data['correction_source_stamp_ns']
            epoch = data['correction_epoch']
            if (isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0
                    or isinstance(epoch, bool) or not isinstance(epoch, int)):
                return
        except (KeyError, TypeError, ValueError):
            return
        self.samples[stamp] = dict(data)
        if len(self.samples) > 32:
            self.samples.pop(next(iter(self.samples)))

    def accept(self, stamp_ns, frame, now):
        data = self.samples.get(stamp_ns, {})
        measured = data.get('correction_measured', {})
        age = now - stamp_ns * 1e-9

        def observed(axis):
            value = measured.get(axis) if isinstance(measured, dict) else None
            return (isinstance(value, (int, float)) and not isinstance(value, bool)
                    and math.isfinite(value) and value > 0.9)

        if self.last_ns is not None and stamp_ns <= self.last_ns:
            self.reason = 'correction is not newer than the last one accepted'
            return False
        valid = (not self.fault and self.current_valid
                 and math.isfinite(age) and -self.future <= age <= self.max_age
                 and frame == 'odom' and data.get('correction_frame') == frame
                 and data.get('correction_valid') is True
                 and data.get('odometry_valid') is True
                 and data.get('correction_status') == 'tracking'
                 and observed('x') and observed('y') and observed('heading'))
        if not valid:
            self.reason = 'correction lacks fresh, matching, observed tracking evidence'
            return False
        epoch = data['correction_epoch']
        if self.epoch is not None and epoch != self.epoch:
            self.reason = 'localization epoch changed'
            return False
        self.epoch = epoch
        self.last_ns = stamp_ns
        self.reason = ''
        return True

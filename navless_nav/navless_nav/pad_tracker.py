"""Where the landing pad is, from every camera fix of the flight (pad_observer's /team/pad/center).

No map prior: the course (and the pad's place on it) can change, so a fix anywhere counts. Fixes are
clustered in odom: one joins the nearest cluster within join_m, else starts its own. The pad is the
cluster with the most fixes, once it has at least `confirm`. A one-frame false detection stays a
cluster of one. A cluster's position is the mean of its latest `keep` fixes, so it follows the VIO
drift between an early sighting and the landing.
"""
import math


class PadTracker:

    def __init__(self, join_m=0.15, confirm=3, keep=10):
        self.join_m, self.confirm, self.keep = join_m, confirm, keep
        self.clusters = []          # [{'fixes': [(t, x, y), ...], 'n': all-time count}]

    @staticmethod
    def _centre(c):
        f = c['fixes']
        return (sum(x for _, x, _ in f) / len(f), sum(y for _, _, y in f) / len(f))

    def add(self, t, x, y):
        near = min(self.clusters, default=None,
                   key=lambda c: math.dist(self._centre(c), (x, y)))
        if near is None or math.dist(self._centre(near), (x, y)) > self.join_m:
            near = {'fixes': [], 'n': 0}
            self.clusters.append(near)
        near['fixes'] = (near['fixes'] + [(t, x, y)])[-self.keep:]
        near['n'] += 1

    def candidate(self):
        """(x, y, fixes, latest) of the most-seen cluster, confirmed or not, or None."""
        c = max(self.clusters, default=None, key=lambda c: c['n'])
        return None if c is None else (*self._centre(c), c['n'], c['fixes'][-1][0])

    def best(self):
        """(x, y, fixes, time of the latest fix) of the confirmed pad, or None."""
        c = max(self.clusters, default=None, key=lambda c: c['n'])
        if c is None or c['n'] < self.confirm:
            return None
        return (*self._centre(c), c['n'], c['fixes'][-1][0])


if __name__ == '__main__':
    tr = PadTracker()
    tr.add(0.0, 5.0, 5.0)                        # a false detection
    assert tr.best() is None
    for i in range(3):
        tr.add(1.0 + i, 1.00 + 0.01 * i, 2.0)
    x, y, n, t = tr.best()
    assert n == 3 and abs(x - 1.01) < 1e-9 and t == 3.0
    for i in range(12):                          # drift: the pad creeps 1 cm a fix
        tr.add(10.0 + i, 1.02 + 0.01 * i, 2.0)
    assert abs(tr.best()[0] - (1.02 + 0.01 * 6.5)) < 1e-9   # latest 10 fixes only
    print('pad_tracker ok')

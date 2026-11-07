import math
import sys
import types

from tof_driver import Smoother, mm_to_range, open_sensor, read_mm, view_text

assert mm_to_range(650) == 0.65
assert math.isinf(mm_to_range(0)) and math.isinf(mm_to_range(-1)) and math.isinf(mm_to_range(None))

# centred: O on the centre line
v = view_text(0.6, 0.6, 30.0)
assert "+0.0 cm" in v and "OK " in v
# closer to the right wall: tube centre is to the left (positive), same sign as tube_node,
# and the drone marker sits right of the centre line
v = view_text(0.4, 0.2, 30.0)
assert "+10.0 cm" in v
row = next(l for l in v.splitlines() if "left wall [" in l)
assert row.index("O") > row.index("|")
# a bad side (prop at 4 cm, or no return) shows BAD and no offset
v = view_text(0.04, 0.6, 30.0)
assert "BAD" in v and "n/a" in v
assert "BAD" in view_text(0.6, math.inf, 30.0)


# A stand-in for the real VL53L1X library with its one quirk that matters here: the C core has ONE global
# pair of I2C callbacks, set by whichever sensor last called _configure_i2c_library_functions() (open() does).
class FakeI2C:
    bus = None
    world = {}                                   # bus -> (mm, range status string)


def install_fake_library():
    mod = types.ModuleType("VL53L1X")

    class VL53L1xUserRoi:
        def __init__(self, *corners):
            self.corners = corners

    class VL53L1X:
        def __init__(self, i2c_bus=1, i2c_address=0x29):
            self.bus, self.roi, self.mode, self.timing = i2c_bus, None, None, None

        def open(self, reset=False):
            self._configure_i2c_library_functions()

        def _configure_i2c_library_functions(self):
            FakeI2C.bus = self.bus

        def set_user_roi(self, roi):
            self.roi = roi.corners

        def start_ranging(self, mode):
            self.mode = mode

        def set_timing(self, budget_us, period_ms):
            self.timing = (budget_us, period_ms)

        def stop_ranging(self):
            pass

        def get_distance(self):                  # answers from whatever bus the GLOBAL callbacks point at
            return FakeI2C.world[FakeI2C.bus][0]

        def get_range_status_string(self):
            return FakeI2C.world[FakeI2C.bus][1]

    mod.VL53L1X, mod.VL53L1xUserRoi = VL53L1X, VL53L1xUserRoi
    sys.modules["VL53L1X"] = mod


def test_each_side_reads_its_own_bus():
    install_fake_library()
    FakeI2C.world = {7: (450, "Range Valid"), 1: (650, "Range Valid")}
    left, right = open_sensor(7), open_sensor(1)          # right opened last: the library now talks to bus 1
    assert left.get_distance() == 650                     # the bug: a plain call reads the other sensor
    assert read_mm(left) == 450 and read_mm(right) == 650 and read_mm(left) == 450


def test_open_sensor_uses_the_whole_array_short_mode_100ms():
    install_fake_library()
    s = open_sensor(7)
    assert s.roi == (0, 15, 15, 0) and s.mode == 1 and s.timing == (100000, 105)


def test_only_readings_marked_valid_count():
    install_fake_library()
    left = open_sensor(7)
    for mm, status in ((69, "Min Range Fail"), (-1, "Signal Fail"), (420, "Phase Fail"), (0, "Range Valid")):
        FakeI2C.world = {7: (mm, status)}
        assert read_mm(left) is None, (mm, status)


def test_smoother_takes_the_median_and_holds_through_a_dropout():
    s = Smoother(3)
    assert s.push(500) == 0.5
    assert s.push(None) == 0.5                            # one bad sample: keep the last good value
    assert s.push(510) == 0.505                           # median of the good ones in the window: 500, 510
    s.push(520)
    assert s.push(900) == 0.52                            # an outlier can't jump it: median of 510, 520, 900
    s.push(None), s.push(None)
    assert math.isinf(s.push(None))                       # three bad in a row: nothing seen
    assert Smoother(1).push(500, offset_mm=-12) == 0.488  # per-sensor correction from the wall test


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
print("ok")

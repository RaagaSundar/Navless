#!/usr/bin/env python3
"""Driver for the two side ToF sensors (TOF400C, ST VL53L1X), one per Jetson I2C bus. Raaga's task.

Feeds tube/tube_node.py, which does the centring math.

Out: /tube/left/range, /tube/right/range   sensor_msgs/Range, ~10 Hz, sensor-data QoS, metres
     Only readings the sensor itself marks valid, as the median of each side's last 3 (one bad sample
     can't jump the value). Nothing valid in the last 3 publishes +inf (REP 117: "nothing seen").
     tube_node still applies its own range limits.
     TF: base_link -> tof_left / tof_right (static), so RViz can draw both beams.
Never publishes to /fmu/in/*.

Wiring (Jetson 40-pin header; pins 4, 6, 8, 10 belong to the Pixhawk, don't touch):
  Left  sensor: VIN pin 1 (3.3V), GND pin 9,  SDA pin 3,  SCL pin 5   -> I2C bus 7
  Right sensor: VIN pin 17 (3.3V), GND pin 25, SDA pin 27, SCL pin 28 -> I2C bus 1
  INT and SHUT unconnected. Never 5 V. Check with: i2cdetect -l; sudo i2cdetect -y -r <bus>  (expect 29)

Install once:  pip3 install smbus2 VL53L1X

  sudo python3 tube/tof_driver.py --view          live terminal view, no ROS (bench test)
  python3 tube/tof_driver.py                      ROS 2: publish the ranges
  python3 tube/tof_driver.py --view --ros         both
  add --left-bus N --right-bus M if i2cdetect showed other bus numbers
  add --left-offset-mm A --right-offset-mm B to correct each sensor after a wall test

Pair with tube_node using the mount offset (sensor face to centreline):
  python3 tube/tube_node.py --ros-args -p left_mount:=0.075 -p right_mount:=0.075 -p min_range:=0.15
"""
import argparse
import math
import signal
import statistics
import sys
import time
from collections import deque

LEFT_BUS, RIGHT_BUS = 7, 1
ADDR = 0x29
VIEW_MIN_M, VIEW_MAX_M = 0.15, 1.30   # only for the terminal view; tube_node has its own limits
FOV_RAD = math.radians(27)            # the whole 16x16 SPAD array (see open_sensor)
BUDGET_US, PERIOD_MS = 100000, 105    # 100 ms per measurement, ~10 Hz
MEDIAN_OF = 3                         # each side publishes the median of its last 3 valid readings
MOUNT_X, MOUNT_Y, MOUNT_Z = -0.019, 0.075, -0.060   # body FLU, metres: 19 mm behind centre, plate edge, below props


def mm_to_range(mm):
    """Driver millimetres -> Range.range metres. Non-positive or missing -> +inf (no return)."""
    if mm is None or mm <= 0:
        return math.inf
    return mm / 1000.0


def view_text(L, R, hz):
    """One frame of the terminal view. L, R in metres (inf = no return)."""
    ok = lambda m: VIEW_MIN_M <= m <= VIEW_MAX_M

    def bar(m, flip=False, width=40):
        s = "?" * 3 + " " * (width - 3) if not ok(m) else "#" * int(m / VIEW_MAX_M * width)
        s = s.ljust(width)
        return s[::-1] if flip else s

    def cm(m):
        return "   --" if math.isinf(m) else f"{m * 100:5.1f}"

    if ok(L) and ok(R):
        off = (L - R) / 2                     # same sign as tube_node: + = centreline is to the drone's LEFT
        meter = [" "] * 41
        meter[20] = "|"
        # O = drone, | = tube centre. off > 0 means the centre is to our left, so we sit right of it.
        meter[20 + int(max(-1.0, min(1.0, off / 0.3)) * 20)] = "O"
        centre, offtxt, width = "".join(meter), f"{off * 100:+6.1f} cm", f"{(L + R) * 100:6.1f} cm"
    else:
        centre, offtxt, width = "      (need both sensors valid)          ", "   n/a", "   n/a"
    return (f"\x1b[2J\x1b[H"
            f"  Navless side ToF   {hz:4.1f} Hz      (Ctrl+C to stop)\n\n"
            f"  LEFT   {cm(L)} cm  {'OK ' if ok(L) else 'BAD'} [{bar(L, flip=True)}]\n"
            f"  RIGHT  {cm(R)} cm  {'OK ' if ok(R) else 'BAD'} [{bar(R)}]\n\n"
            f"  width L+R  {width}\n"
            f"  offset     {offtxt}   (+ = tube centre is to the drone's left)\n\n"
            f"  left wall [{centre}] right wall\n")


def use(sensor):
    """Point the VL53L1X library at this sensor before talking to it.

    The library's C core keeps ONE global pair of I2C read/write callbacks, set by whichever sensor was
    opened last. With one sensor per bus, every call would otherwise go to the last-opened bus, so both
    sides read the right sensor (seen on skybeta-19: LEFT and RIGHT both 8.1 cm)."""
    sensor._configure_i2c_library_functions()


def open_sensor(bus):
    import VL53L1X
    s = VL53L1X.VL53L1X(i2c_bus=bus, i2c_address=ADDR)
    s.open()                                               # also points the library's I2C callbacks here
    # Bench on skybeta-19, white target: the old 8x8 centre with 33 ms read to ~30 cm; the whole array with
    # 100 ms to ~60 cm on paper and ~90 cm on a large surface. Medium and long modes failed in room light.
    s.set_user_roi(VL53L1X.VL53L1xUserRoi(0, 15, 15, 0))  # whole 16x16 SPAD array, ~27 deg cone: keep props out
    s.start_ranging(1)                                     # short mode
    s.set_timing(BUDGET_US, PERIOD_MS)
    return s


def read_mm(sensor):
    """One reading in mm, or None unless the sensor marked it valid. The library still returns a distance
    for some rejected readings (a "Min Range Fail" came back as 69 mm on the bench)."""
    use(sensor)
    try:
        mm = sensor.get_distance()
        status = (sensor.get_range_status_string() if hasattr(sensor, "get_range_status_string")
                  else "Range Valid")             # older library versions only return -1 when rejected
    except OSError:
        return None
    return mm if mm is not None and mm > 0 and status == "Range Valid" else None


class Smoother:
    """Median of a side's last N readings that were valid; +inf once none of the last N were."""

    def __init__(self, n=MEDIAN_OF):
        self.window = deque(maxlen=max(1, n))

    def push(self, mm, offset_mm=0.0):
        self.window.append(None if mm is None else mm + offset_mm)
        good = [v for v in self.window if v is not None]
        return mm_to_range(statistics.median(good)) if good else math.inf


def main():
    ap = argparse.ArgumentParser(description="Side ToF driver for tube_node")
    ap.add_argument("--view", action="store_true", help="live terminal view")
    ap.add_argument("--ros", action="store_true", help="also publish ROS 2 topics when using --view")
    ap.add_argument("--left-bus", type=int, default=LEFT_BUS)
    ap.add_argument("--right-bus", type=int, default=RIGHT_BUS)
    ap.add_argument("--left-offset-mm", type=float, default=0.0, help="added to every left reading (wall test)")
    ap.add_argument("--right-offset-mm", type=float, default=0.0, help="added to every right reading (wall test)")
    ap.add_argument("--median", type=int, default=MEDIAN_OF, help="median of the last N valid readings (1 = raw)")
    args, _ = ap.parse_known_args()   # tolerate --ros-args
    use_ros = args.ros or not args.view

    left, right = open_sensor(args.left_bus), open_sensor(args.right_bus)

    node = None
    if use_ros:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Range
        from geometry_msgs.msg import TransformStamped
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        rclpy.init()
        node = Node("tof_driver")
        pubs = {s: node.create_publisher(Range, f"/tube/{s}/range", qos_profile_sensor_data) for s in ("left", "right")}
        tfs = []
        for name, y, yaw in (("tof_left", MOUNT_Y, math.pi / 2), ("tof_right", -MOUNT_Y, -math.pi / 2)):
            t = TransformStamped()
            t.header.stamp = node.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = "base_link", name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = MOUNT_X, y, MOUNT_Z
            t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            tfs.append(t)
        tf_broadcaster = StaticTransformBroadcaster(node)
        tf_broadcaster.sendTransform(tfs)

        def publish(side, m, stamp):
            r = Range()
            r.header.stamp, r.header.frame_id = stamp, f"tof_{side}"
            r.radiation_type = Range.INFRARED
            r.field_of_view = FOV_RAD
            r.min_range, r.max_range = 0.04, 4.0          # sensor's physical limits
            r.range = float(m)
            pubs[side].publish(r)

    # Ctrl+C / the launch's SIGTERM set a flag: the library swallows KeyboardInterrupt inside its I2C
    # callbacks, so a plain except KeyboardInterrupt never stopped the loop. Installed after rclpy.init().
    stop = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.append(True))

    smooth = {"left": Smoother(args.median), "right": Smoother(args.median)}
    t_last, hz = time.time(), 0.0
    try:
        while not stop:
            L = smooth["left"].push(read_mm(left), args.left_offset_mm)
            R = smooth["right"].push(read_mm(right), args.right_offset_mm)
            now = time.time()
            hz = 0.9 * hz + 0.1 / max(now - t_last, 1e-3)
            t_last = now
            if node is not None:
                stamp = node.get_clock().now().to_msg()
                publish("left", L, stamp)
                publish("right", R, stamp)
                rclpy.spin_once(node, timeout_sec=0)
            if args.view:
                sys.stdout.write(view_text(L, R, hz))
                sys.stdout.flush()
            time.sleep(0.005)                       # get_distance() already waits for each measurement
    except KeyboardInterrupt:
        pass
    finally:
        for s in (left, right):
            try:
                use(s)
                s.stop_ranging()
            except OSError:
                pass
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()

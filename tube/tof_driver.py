#!/usr/bin/env python3
"""Driver for the two side ToF sensors (TOF400C, ST VL53L1X), one per Jetson I2C bus. Raaga's task.

Feeds tube/tube_node.py, which does the centring math.

Out: /tube/left/range, /tube/right/range   sensor_msgs/Range, ~30 Hz, sensor-data QoS, metres
     Raw readings (tube_node does the rejecting). A failed read publishes +inf (REP 117: "nothing seen").
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

Pair with tube_node using the mount offset (sensor face to centreline):
  python3 tube/tube_node.py --ros-args -p left_mount:=0.075 -p right_mount:=0.075 -p min_range:=0.15
"""
import argparse
import math
import sys
import time

LEFT_BUS, RIGHT_BUS = 7, 1
ADDR = 0x29
VIEW_MIN_M, VIEW_MAX_M = 0.15, 1.30   # only for the terminal view; tube_node has its own limits
FOV_RAD = math.radians(15)            # beam after the ROI below
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


def open_sensor(bus):
    import VL53L1X
    s = VL53L1X.VL53L1X(i2c_bus=bus, i2c_address=ADDR)
    s.open()
    s.set_user_roi(VL53L1X.VL53L1xUserRoi(4, 11, 11, 4))  # centre 8x8 of 16x16 SPADs: ~15 deg cone, clears the props
    s.start_ranging(1)                                     # short mode: best in room light, up to ~1.3 m
    s.set_timing(33000, 34)                                # 33 ms budget, ~30 Hz
    return s


def main():
    ap = argparse.ArgumentParser(description="Side ToF driver for tube_node")
    ap.add_argument("--view", action="store_true", help="live terminal view")
    ap.add_argument("--ros", action="store_true", help="also publish ROS 2 topics when using --view")
    ap.add_argument("--left-bus", type=int, default=LEFT_BUS)
    ap.add_argument("--right-bus", type=int, default=RIGHT_BUS)
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

    t_last, hz = time.time(), 0.0
    try:
        while True:
            try:
                L = mm_to_range(left.get_distance())
            except OSError:
                L = math.inf
            try:
                R = mm_to_range(right.get_distance())
            except OSError:
                R = math.inf
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
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        left.stop_ranging()
        right.stop_ranging()
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()

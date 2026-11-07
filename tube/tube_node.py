#!/usr/bin/env python3
"""Tube centring from the left and right distance sensors (Raaga's task, picked up by @sim).

In:  /tube/left/range, /tube/right/range   sensor_msgs/Range, from whatever driver or sim model
Out: /tube/offset   geometry_msgs/PointStamped, frame base_link (FLU), 30 Hz
     point.y = (L - R) / 2   how far the tube centreline is to the drone's LEFT, metres
     point.z = L + R         measured tube width, metres (sanity check)
Nothing is published unless both readings are fresh and believable: silence means "don't trust".
Never publishes to /fmu/in/*.

  python3 tube/tube_node.py --ros-args -p use_sim_time:=true -p min_range:=0.12

Knobs to set on the real drone:
  min_range        readings closer than this are rejected. Raise it until spinning each prop
                   slowly by hand never produces a published reading (prop check).
  left_mount, right_mount   distance from the drone's centreline to each sensor face, metres.
  expected_width   tube inner width; readings with |L+R - width| > width_tol are dropped
                   (0 = check off). Catches a prop hit on one side, or the gap at the tube exit.
"""

STALE_S = 0.1  # 3 frames at 30 Hz


def tube_offset(left, right, min_range, max_range, left_mount=0.0, right_mount=0.0,
                expected_width=0.0, width_tol=0.15):
    """Return (offset_left_m, width_m), or None if the pair can't be trusted."""
    if not (min_range <= left <= max_range and min_range <= right <= max_range):  # also drops NaN/inf
        return None
    L, R = left + left_mount, right + right_mount
    if expected_width > 0 and abs(L + R - expected_width) > width_tol:
        return None
    # ponytail: assumes the drone is roughly aligned with the tube; a yaw error e stretches L+R by 1/cos(e)
    return (L - R) / 2, L + R


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Range
    from geometry_msgs.msg import PointStamped

    class Tube(Node):
        def __init__(self):
            super().__init__("tube_offset")
            p = lambda n, v: self.declare_parameter(n, v).value
            self.cfg = dict(min_range=p("min_range", 0.10), max_range=p("max_range", 2.0),
                            left_mount=p("left_mount", 0.0), right_mount=p("right_mount", 0.0),
                            expected_width=p("expected_width", 0.0), width_tol=p("width_tol", 0.15))
            self.last = {}
            for side in ("left", "right"):
                self.create_subscription(Range, f"/tube/{side}/range",
                                         lambda m, s=side: self.last.__setitem__(s, (m.range, self.get_clock().now())),
                                         qos_profile_sensor_data)
            self.pub = self.create_publisher(PointStamped, "/tube/offset", qos_profile_sensor_data)
            self.create_timer(1.0 / 30.0, self.tick)

        def tick(self):
            now = self.get_clock().now()
            if any(s not in self.last or (now - self.last[s][1]).nanoseconds > STALE_S * 1e9 for s in ("left", "right")):
                return
            r = tube_offset(self.last["left"][0], self.last["right"][0], **self.cfg)
            if r is None:
                return
            m = PointStamped()
            m.header.stamp, m.header.frame_id = now.to_msg(), "base_link"
            m.point.y, m.point.z = float(r[0]), float(r[1])
            self.pub.publish(m)

    rclpy.init()
    node = Tube()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

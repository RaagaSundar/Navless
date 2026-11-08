"""Calibrated stereo producer for /team/stereo/points on aircraft 19."""
import math
import time
import cv2
import numpy as np

class StereoDepth:
    """Rectify a stereo pair, run SGBM, return depth and 3D points in the LEFT optical frame."""

    def __init__(self, num_disp=64, block=5, min_depth=0.3, max_depth=5.0, texture_filter=False):
        self.num_disp = int(math.ceil(num_disp / 16) * 16)
        self.min_depth, self.max_depth = min_depth, max_depth
        self.texture_filter = texture_filter
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=self.num_disp, blockSize=block,
            P1=8 * block * block, P2=32 * block * block, disp12MaxDiff=1,
            uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self._key = None

    def setup(self, K1, D1, K2, D2, size, R, T):
        """K/D per camera at working resolution; R,T map LEFT-camera points into the RIGHT camera:
        X_r = R X_l + T (for a side-by-side pair, T ≈ [-baseline, 0, 0])."""
        key = (K1.tobytes(), D1.tobytes(), K2.tobytes(), D2.tobytes(), size, R.tobytes(), T.tobytes())
        if key == self._key:
            return
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, size, R, T,
                                                    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.map_l = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_16SC2)
        self.Q, self.R1 = Q, R1
        self.fx, self.baseline = P1[0, 0], float(np.linalg.norm(T))
        self._key = key

    def compute(self, gray_l, gray_r):
        rl = cv2.remap(gray_l, *self.map_l, cv2.INTER_LINEAR)
        rr = cv2.remap(gray_r, *self.map_r, cv2.INTER_LINEAR)
        disp = self.sgbm.compute(rl, rr).astype(np.float32) / 16.0
        xyz_rect = cv2.reprojectImageTo3D(disp, self.Q)               # rectified-left frame
        z = xyz_rect[..., 2]
        valid = (disp > 0.5) & np.isfinite(z) & (z > self.min_depth) & (z < self.max_depth)
        if self.texture_filter:
            # SGBM can propagate a rim disparity through an untextured opening.
            # Such pixels are not independent depth measurements for wall fitting.
            intensity = rl.astype(np.float32)
            mean = cv2.boxFilter(intensity, -1, (5, 5))
            variance = cv2.boxFilter(intensity*intensity, -1, (5, 5)) - mean*mean
            valid &= variance > 30.0
        depth = np.where(valid, z, 0.0).astype(np.float32)
        pts = xyz_rect[valid] @ self.R1                                # rect -> left optical (R1^T)
        return rl, depth, valid, pts.astype(np.float32)


def gray(msg):
    channels = {'mono8':1, 'bgr8':3, 'rgb8':3}[msg.encoding]
    if msg.step < msg.width*channels or len(msg.data) != msg.height*msg.step:
        raise ValueError('invalid image stride')
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    pixels = image[:, :msg.width*channels]
    if channels == 1:
        return pixels
    conversion = cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(pixels.reshape(msg.height, msg.width, 3), conversion)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    import message_filters
    from tf2_ros import Buffer, TransformListener, TransformException
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
    from cloud_projection import rotation, xyzw, xyz, stamp_s

    class Stereo(Node):
        def __init__(self):
            super().__init__('real19_stereo_cloud')
            self.depth = StereoDepth(max_depth=2.8, texture_filter=True)
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.out = self.create_publisher(PointCloud2, '/team/stereo/points', 1)
            self.last = 0.
            inputs = [message_filters.Subscriber(self, typ, f'/oak/{side}/{suffix}', qos_profile=qos_profile_sensor_data)
                      for typ, suffix in ((Image,'image_raw'), (CameraInfo,'camera_info'))
                      for side in ('left','right')]
            self.sync = message_filters.ApproximateTimeSynchronizer(inputs, queue_size=6, slop=.01)
            self.sync.registerCallback(self.pair)

        def pair(self, left, right, lc, rc):
            now = time.monotonic()
            if now - self.last < .2:
                return
            self.last = now
            try:
                size = (left.width//2, left.height//2)
                scale = np.diag([size[0]/left.width, size[1]/left.height, 1.])
                if left.width != right.width or left.height != right.height or left.header.frame_id == right.header.frame_id:
                    raise ValueError('incompatible stereo images')
                t = self.get_clock().now().nanoseconds*1e-9
                for im, info in ((left,lc), (right,rc)):
                    if (not im.header.frame_id or im.header.frame_id != info.header.frame_id
                            or (im.width,im.height) != (info.width,info.height)
                            or not 0 <= t-stamp_s(im.header) <= .25
                            or info.distortion_model not in ('plumb_bob','rational_polynomial')
                            or not np.isfinite([*info.k,*info.d]).all() or min(info.k[0],info.k[4]) <= 0):
                        raise ValueError('invalid stereo calibration or stamp')
                tf = self.tf.lookup_transform(right.header.frame_id,left.header.frame_id,Time.from_msg(left.header.stamp))
                R, T = rotation(xyzw(tf.transform.rotation)), xyz(tf.transform.translation)
                if np.linalg.norm(T) <= .001:
                    raise ValueError('invalid stereo baseline')
                self.depth.setup(scale@np.reshape(lc.k,(3,3)),np.asarray(lc.d),
                                 scale@np.reshape(rc.k,(3,3)),np.asarray(rc.d),size,R,T.reshape(3,1))
                _, _, _, pts = self.depth.compute(cv2.resize(gray(left),size),cv2.resize(gray(right),size))
                if len(pts):
                    self.out.publish(create_cloud_xyz32(left.header, pts[::4]))
            except (ValueError, KeyError, cv2.error, TransformException) as exc:
                self.get_logger().warn(str(exc), throttle_duration_sec=2.)

    cv2.setNumThreads(1)
    rclpy.init()
    node = Stereo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

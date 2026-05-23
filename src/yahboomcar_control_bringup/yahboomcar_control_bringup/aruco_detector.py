"""
aruco_detector.py

Detects ArUco marker IDs 0, 1, and 2 in the robot's camera feed.
Uses PnP solver and static offsets to estimate the 3D pose of the Dock Center.
Implements:
1. Priority-based selection (ID 0 Center > ID 1/2 Both > ID 1 Left > ID 2 Right)
   to prevent pose jumping when side markers flicker.
2. Exponential Moving Average (EMA) filtering to smooth out tracking noise.
Only processes camera images when activated via /dock/enable_detector
OR when there are active subscribers to /dock/pose_3d or /dock/debug_image.
Publishes annotated video stream to /dock/debug_image (headless mode)
and 3D pose to /dock/pose_3d.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

# ArUco dictionary — DICT_4X4_50
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

# Marker configuration
MARKER_SIZE = 0.08  # meters (8cm)
MARKER_CORNERS_3D = np.array([
    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0.0],
    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0.0],
    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0.0],
    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0.0]
], dtype=np.float32)

# Static offsets: T_marker_to_dock (pose of marker relative to the Dock Center Frame)
def make_transform(x, y, z, yaw_deg):
    yaw = np.radians(yaw_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [c,   -s,  0.0, x],
        [s,    c,  0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ])

# Math helpers for quaternion rotations
def rotation_matrix_to_quaternion(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)

def quaternion_to_rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,         2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,         1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,         1 - 2*qx**2 - 2*qy**2]
    ])


class ArucoDetector(Node):
    def __init__(self):
        super().__init__(
            'aruco_detector',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.bridge = CvBridge()
        self.active = False  # Disabled by default to save CPU
        self.K = None        # Intrinsics
        self.D = None        # Distortion
        self.last_log_time = 0.0  # For throttled terminal logs
        self._last_marker_status = None  # (best_id, visible_mask) for change detection

        # ── Marker geometry (config-driven) ─────────────────────────────────
        # Stored as dock->marker transforms (pose of each marker in Dock frame).
        # This must match the dock geometry in the SDF.
        dock_id = str(self.get_parameter('dock_id').value) if self.has_parameter('dock_id') else 'default'

        def _load_marker_tf(marker_key: str, default_x: float, default_y: float, default_z: float, default_yaw_deg: float):
            prefix = f'docks.{dock_id}.markers.{marker_key}'
            try:
                x = float(self.get_parameter(f'{prefix}.x').value)
                y = float(self.get_parameter(f'{prefix}.y').value)
                z = float(self.get_parameter(f'{prefix}.z').value)
                yaw = float(self.get_parameter(f'{prefix}.yaw_deg').value)
            except Exception:
                x, y, z, yaw = default_x, default_y, default_z, default_yaw_deg
            return make_transform(x, y, z, yaw)

        # Defaults are kept for safety, but should be overridden by dock.yaml.
        self.T_dock_to_m0 = _load_marker_tf('m0', 0.0, 0.0, 0.0, 0.0)
        self.T_dock_to_m1 = _load_marker_tf('m1', 0.042, -0.085, 0.0, 45.0)
        self.T_dock_to_m2 = _load_marker_tf('m2', 0.042, 0.085, 0.0, -45.0)

        self.get_logger().info(
            f'Dock marker geometry loaded for dock_id="{dock_id}": '
            f'm0(x={self.T_dock_to_m0[0,3]:.3f}, y={self.T_dock_to_m0[1,3]:.3f}), '
            f'm1(x={self.T_dock_to_m1[0,3]:.3f}, y={self.T_dock_to_m1[1,3]:.3f}), '
            f'm2(x={self.T_dock_to_m2[0,3]:.3f}, y={self.T_dock_to_m2[1,3]:.3f})'
        )

        # Exponential Moving Average (EMA) low-pass filter states
        self.filtered_t = None
        self.filtered_q = None
        self.ema_alpha = 0.1  # Smoothing factor (lower = smoother, higher = faster response)

        # Subscribe to enable/disable topic
        self.enable_sub = self.create_subscription(
            Bool,
            '/dock/enable_detector',
            self.enable_callback,
            10
        )

        # Subscribe to camera info for intrinsics
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )

        # Subscribe to camera images
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish 3D synthesized Dock Center pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/dock/pose_3d',
            10
        )

        # Publish marker visibility status for coordinator state logic
        # msg.data = [best_id, visible_mask]
        # best_id: -1 (none), 0 (center), 1 (left), 2 (right), 3 (left+right only)
        # visible_mask: bit0=id0, bit1=id1, bit2=id2
        self.status_pub = self.create_publisher(
            Int32MultiArray,
            '/dock/marker_status',
            10
        )

        # Publish debug annotated image
        self.debug_pub = self.create_publisher(
            Image,
            '/dock/debug_image',
            10
        )

        self.get_logger().info(
            'ArUco Detector initialized (Dormant). '
            'Waiting for activation and camera info...'
        )

    def enable_callback(self, msg: Bool):
        self.active = msg.data
        self.get_logger().info(f'ArUco Detector state changed. Active: {self.active}')
        if not self.active:
            # Reset filter when deactivated
            self.filtered_t = None
            self.filtered_q = None

    def info_callback(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape((3, 3))
            self.D = np.array(msg.d)
            self.get_logger().info('Camera calibration parameters loaded successfully.')
            # Unsubscribe to save bandwidth/CPU
            self.destroy_subscription(self.info_sub)

    def image_callback(self, msg: Image):
        # Auto-activate if there is an active subscriber to debug_image or pose_3d,
        # or if manually enabled via topic.
        has_subscribers = (self.pose_pub.get_subscription_count() > 0) or (self.debug_pub.get_subscription_count() > 0)
        
        if not (self.active or has_subscribers) or self.K is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        debug = frame.copy()

        # Run marker detection
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, ARUCO_DICT, parameters=ARUCO_PARAMS
        )

        detected_poses = {}
        current_time = time.perf_counter()
        log_throttled = (current_time - self.last_log_time) > 1.0

        visible_mask = 0
        best_id = -1

        if ids is not None:
            ids_flat = ids.flatten()
            valid_ids = [mid for mid in ids_flat if mid in [0, 1, 2]]
            visible_mask = (1 if 0 in valid_ids else 0) | (2 if 1 in valid_ids else 0) | (4 if 2 in valid_ids else 0)
            if 0 in valid_ids:
                best_id = 0
            elif 1 in valid_ids and 2 in valid_ids:
                best_id = 3
            elif 1 in valid_ids:
                best_id = 1
            elif 2 in valid_ids:
                best_id = 2
            
            if log_throttled:
                if len(valid_ids) > 0:
                    self.get_logger().info(f"OpenCV Status: DETECTED. Visible Marker IDs: {valid_ids}")
                else:
                    self.get_logger().info(f"OpenCV Status: NO DOCK MARKERS in view (Detected other IDs: {list(ids_flat)})")
                self.last_log_time = current_time

            for i, mid in enumerate(ids_flat):
                if mid in [0, 1, 2]:
                    corners_2d = corners[i][0].astype(np.float32)
                    success, rvec, tvec = cv2.solvePnP(
                        MARKER_CORNERS_3D,
                        corners_2d,
                        self.K,
                        self.D,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    if success:
                        R, _ = cv2.Rodrigues(rvec)
                        T_m_to_c = np.eye(4)
                        T_m_to_c[0:3, 0:3] = R
                        T_m_to_c[0:3, 3] = tvec.flatten()

                        # Apply static offset from Dock Frame to Marker Frame
                        if mid == 0:
                            T_dock_to_c = np.dot(T_m_to_c, np.linalg.inv(self.T_dock_to_m0))
                        elif mid == 1:
                            T_dock_to_c = np.dot(T_m_to_c, np.linalg.inv(self.T_dock_to_m1))
                        elif mid == 2:
                            T_dock_to_c = np.dot(T_m_to_c, np.linalg.inv(self.T_dock_to_m2))

                        detected_poses[mid] = T_dock_to_c

                        # Draw individual marker coordinate axes
                        cv2.drawFrameAxes(debug, self.K, self.D, rvec, tvec, 0.05)
        else:
            if log_throttled:
                self.get_logger().info("OpenCV Status: NOT DETECTED. No markers found in image frame.")
                self.last_log_time = current_time

        # Publish marker visibility status (throttled only by the callback rate)
        # Useful for coordinator state gating; avoids inferring visibility from fused pose.
        status = (best_id, visible_mask)
        if status != self._last_marker_status:
            self._last_marker_status = status
        status_msg = Int32MultiArray()
        status_msg.data = [int(best_id), int(visible_mask)]
        self.status_pub.publish(status_msg)

        # ── Priority-Based Target Selection & Fusion ──────────────────────────
        t_target = None
        R_target = None

        if 0 in detected_poses:
            # Priority 1: Center Marker (ID 0) is visible. Use its pose directly.
            # This completely avoids coordinate jumps when side markers flicker.
            t_target = detected_poses[0][0:3, 3]
            R_target = detected_poses[0][0:3, 0:3]
        elif 1 in detected_poses and 2 in detected_poses:
            # Priority 2: Both side markers are visible. Average them.
            translations = [detected_poses[1][0:3, 3], detected_poses[2][0:3, 3]]
            t_target = np.mean(translations, axis=0)

            q1 = rotation_matrix_to_quaternion(detected_poses[1][0:3, 0:3])
            q2 = rotation_matrix_to_quaternion(detected_poses[2][0:3, 0:3])
            # Handle antipodal alignment
            if np.dot(q1, q2) < 0.0:
                q2 = -q2
            q_target = (q1 + q2) / 2.0
            q_target /= np.linalg.norm(q_target)
            R_target = quaternion_to_rotation_matrix(q_target)
        elif 1 in detected_poses:
            # Priority 3: Only Left Marker is visible.
            t_target = detected_poses[1][0:3, 3]
            R_target = detected_poses[1][0:3, 0:3]
        elif 2 in detected_poses:
            # Priority 4: Only Right Marker is visible.
            t_target = detected_poses[2][0:3, 3]
            R_target = detected_poses[2][0:3, 0:3]

        # ── Exponential Moving Average (EMA) Filtering ───────────────────────
        if t_target is not None and R_target is not None:
            q_target = rotation_matrix_to_quaternion(R_target)

            if self.filtered_t is None or self.filtered_q is None:
                self.filtered_t = t_target
                self.filtered_q = q_target
            else:
                # Filter translation
                self.filtered_t = self.ema_alpha * t_target + (1.0 - self.ema_alpha) * self.filtered_t
                
                # Filter rotation (quaternion LERP with antipodal alignment check)
                if np.dot(q_target, self.filtered_q) < 0.0:
                    q_target = -q_target
                q_next = self.ema_alpha * q_target + (1.0 - self.ema_alpha) * self.filtered_q
                self.filtered_q = q_next / np.linalg.norm(q_next)

            R_filtered = quaternion_to_rotation_matrix(self.filtered_q)

            # Synthesize final Pose in camera optical frame
            T_dock_to_c = np.eye(4)
            T_dock_to_c[0:3, 0:3] = R_filtered
            T_dock_to_c[0:3, 3] = self.filtered_t

            # Transform from camera optical frame (Z forward, X right, Y down)
            # to camera joint frame (X forward, Y left, Z up) matching TF tree link
            T_opt_to_joint = np.array([
                [0.0,  0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0,  0.0, 0.0, 1.0]
            ])
            T_dock_to_joint = np.dot(T_opt_to_joint, T_dock_to_c)

            t_joint = T_dock_to_joint[0:3, 3]
            R_joint = T_dock_to_joint[0:3, 0:3]
            q_final = rotation_matrix_to_quaternion(R_joint)

            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = msg.header.frame_id if msg.header.frame_id else "jq2_Link"

            pose_msg.pose.position.x = float(t_joint[0])
            pose_msg.pose.position.y = float(t_joint[1])
            pose_msg.pose.position.z = float(t_joint[2])

            pose_msg.pose.orientation.x = float(q_final[0])
            pose_msg.pose.orientation.y = float(q_final[1])
            pose_msg.pose.orientation.z = float(q_final[2])
            pose_msg.pose.orientation.w = float(q_final[3])

            self.pose_pub.publish(pose_msg)

            # Draw filtered pose axes in optical coordinates for visual validation
            rvec_fused, _ = cv2.Rodrigues(R_filtered)
            cv2.drawFrameAxes(debug, self.K, self.D, rvec_fused, self.filtered_t, 0.1)

            cv2.putText(debug, f'FUSED DOCK DETECTED (M: {len(detected_poses)})', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # Reset filter states if tracking is fully lost
            self.filtered_t = None
            self.filtered_q = None
            cv2.putText(debug, 'NO DOCK MARKERS DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Publish debug image
        try:
            ros_image = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            self.debug_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

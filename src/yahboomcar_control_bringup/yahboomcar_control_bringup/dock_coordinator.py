#!/usr/bin/env python3
"""
dock_coordinator.py

Autonomous docking mission coordinator for a differential drive robot.
Exposes a ROS 2 Action Server '/dock'.
Handover pattern:
1. STAGING   - Bypassed if marker is already visible. Otherwise, sends goal to Nav2
               using ActionClient to reach a pose in front of the dock.
2. SEARCHING - Camera sweep / body spin to find the ArUco markers.
3. TRACKING  - Precise orientation and pan/tilt alignment in place.
4. APPROACH  - Visual servoing (S-Curve Pure Pursuit) to the dock center line.
5. INTERLOCK - Bypasses active steering inside the funnel (z_d < 0.45m) and drives
               straight forward.
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64MultiArray, Int32MultiArray
from nav2_msgs.action import NavigateToPose
from yahboomcar_msgs.action import Dock
import tf2_ros
from ament_index_python.packages import get_package_share_directory

# ── Tuning constants ───────────────────────────────────────────────────────────
SEARCH_SPIN_SPEED = 0.6     # rad/s body spin during SEARCHING
SEARCH_TIMEOUT   = 30.0     # Seconds before SEARCHING gives up
LOSS_FULL_TIME   = 1.0      # Seconds lost before regressing to SEARCHING or LOST
LOCK_DURATION    = 0.5      # Seconds aligned before transitioning to APPROACH
DOCK_LIDAR_DIST  = 0.35     # m — forward lidar threshold for DOCKED
KP_PAN           = 0.5      # Proportional gain for head panning
KP_TILT          = 0.5      # Proportional gain for head tilting
# ──────────────────────────────────────────────────────────────────────────────

# Quaternion math helpers
def multiply_quaternions(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])

def quaternion_to_rotation_matrix(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,         2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,         1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw,         1 - 2*qx**2 - 2*qy**2]
    ])


class DockCoordinator(Node):

    def __init__(self):
        super().__init__(
            'dock_coordinator',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        self.cb_group = ReentrantCallbackGroup()

        # ── State gating params (safe defaults) ──────────────────────────────
        # Debounce for fleeting center-marker frames before entering TRACKING.
        self.declare_parameter('center_lock_time_s', 0.3)
        # Stop and settle time to bleed off commanded angular momentum.
        self.declare_parameter('settle_time_s', 0.15)
        # Side-marker-based correction behavior (diff drive safe: rotate-in-place).
        self.declare_parameter('staging_correction_timeout_s', 8.0)
        self.declare_parameter('staging_correction_spin_speed', 0.4)  # rad/s
        # Spin direction when only one side marker is visible.
        # Default heuristic: see left marker -> rotate right (negative), see right -> rotate left (positive).
        self.declare_parameter('staging_correction_omega_sign_left', -1.0)
        self.declare_parameter('staging_correction_omega_sign_right', 1.0)

        # ── TF2 Buffer and Listener ───────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Action Clients ────────────────────────────────────────────────────
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        # ── Action Server ─────────────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            Dock,
            'dock',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.vel_pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10)
        self.head_pub = self.create_publisher(
            Float64MultiArray, '/camera_controller/commands', 10)
        self.enable_pub = self.create_publisher(
            Bool, '/dock/enable_detector', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/dock/pose_3d', self._pose_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            Int32MultiArray, '/dock/marker_status', self._marker_status_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10, callback_group=self.cb_group)

        # ── Shared Sensor State ───────────────────────────────────────────────
        self.dock_pose_c = None  # PoseStamped in camera joint frame
        self.last_pose_t = 0.0   # perf_counter timestamp of last pose
        self.lidar_front = 10.0   # m
        self.marker_best_id = -1
        self.marker_visible_mask = 0
        self._last_marker_log_t = 0.0

        # ── Internal servo state ──────────────────────────────────────────────
        self._pan_angle  = 0.0
        self._tilt_angle = 0.0
        self._last_twist = Twist()

        # ── Center-marker debounce / settle state ────────────────────────────
        self._center_seen_start_t = None
        self._settle_start_t = None

        self.get_logger().info('Dock Action Server is ONLINE and IDLE.')

    def _resolve_staging_bt_xml(self, controller_choice: str) -> str:
        """
        Resolve a behavior tree XML for NavigateToPose that pins the controller_id.
        Nav2 Humble supports overriding BT via NavigateToPose.Goal.behavior_tree.
        """
        choice = (controller_choice or '').strip().lower()
        bringup_share = get_package_share_directory('yahboomcar_control_bringup')
        config_dir = f'{bringup_share}/config'

        if choice in ('mppi', 'nav2_mppi'):
            return f'{config_dir}/nav2_mppi_bt.xml'
        if choice in ('rpp', 'purepursuit', 'regulated_pure_pursuit'):
            return f'{config_dir}/nav2_rpp_bt.xml'
        if choice in ('dwb', 'followpath', 'dwb_local_planner'):
            return f'{config_dir}/nav2_dwb_bt.xml'

        return ''

    # ══════════════════════════════════════════════════════════════════════════
    # Action Server Callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received dock request for ID: {goal_request.dock_id}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Docking action CANCELLED by client.')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """The main mission loop wrapper to handle CV activation/deactivation."""
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)
        self.get_logger().info('Activating ArUco Detector for docking mission.')
        
        try:
            return await self._execute_callback_impl(goal_handle)
        finally:
            disable_msg = Bool()
            disable_msg.data = False
            self.enable_pub.publish(disable_msg)
            self.get_logger().info('Putting ArUco Detector back to sleep.')

    async def _execute_callback_impl(self, goal_handle):
        """The main mission loop."""
        result = Dock.Result()
        feedback = Dock.Feedback()
        
        dock_id = goal_handle.request.dock_id
        self.get_logger().info(f'Starting mission for dock: {dock_id}')

        # 1. Look up staging parameters
        try:
            prefix = f'docks.{dock_id}'
            sx = self.get_parameter(f'{prefix}.staging_x').value
            sy = self.get_parameter(f'{prefix}.staging_y').value
            syaw = self.get_parameter(f'{prefix}.staging_yaw').value
            staging_controller = self.get_parameter(f'{prefix}.staging_controller').value
        except Exception as e:
            self.get_logger().error(f'Dock ID "{dock_id}" not found in config: {e}')
            result.success = False
            result.message = f"Unknown dock ID: {dock_id}"
            goal_handle.abort()
            return result

        # 2. Check for Conditional Staging (bypassing Nav2 if marker is already visible)
        self.dock_pose_c = None
        self.get_logger().info('Checking if dock is already visible...')
        time.sleep(0.5)  # Wait for a couple of frames
        
        pose_fresh = (self.dock_pose_c is not None) and (time.perf_counter() - self.last_pose_t < LOSS_FULL_TIME)

        if pose_fresh and self.marker_best_id == 0:
            # Still apply debounce/settle logic; start in SEARCHING so we don't
            # immediately enter TRACKING on a single fleeting frame.
            self.get_logger().info('Center marker visible at start. Bypassing Nav2 staging phase!')
            current_state = "SEARCHING"
        else:
            # 3. Nav2 Staging Approach
            self.get_logger().info('Dock not visible. Sending goal to Nav2...')
            feedback.state = "STAGING"
            goal_handle.publish_feedback(feedback)

            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('NavigateToPose action server not available! Aborting.')
                result.success = False
                result.message = "Nav2 not available"
                goal_handle.abort()
                return result

            staging_goal = NavigateToPose.Goal()
            staging_goal.pose.header.frame_id = 'map'
            staging_goal.pose.header.stamp = self.get_clock().now().to_msg()
            staging_goal.pose.pose.position.x = sx
            staging_goal.pose.pose.position.y = sy
            staging_goal.pose.pose.orientation.z = math.sin(syaw / 2.0)
            staging_goal.pose.pose.orientation.w = math.cos(syaw / 2.0)

            bt_xml = self._resolve_staging_bt_xml(str(staging_controller))
            if bt_xml:
                staging_goal.behavior_tree = bt_xml
                self.get_logger().info(f'STAGING controller pinned: "{staging_controller}" ({bt_xml})')
            else:
                self.get_logger().info(f'STAGING controller using Nav2 default BT (choice="{staging_controller}")')

            send_goal_future = self.nav_client.send_goal_async(staging_goal)
            
            while not send_goal_future.done():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._stop_robot()
                    result.success = False
                    result.message = "Cancelled during staging goal submission"
                    return result
                time.sleep(0.1)

            nav_goal_handle = send_goal_future.result()
            if not nav_goal_handle.accepted:
                self.get_logger().error('Staging goal rejected by Nav2!')
                result.success = False
                result.message = "Staging goal rejected"
                goal_handle.abort()
                return result

            get_result_future = nav_goal_handle.get_result_async()

            while not get_result_future.done():
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('Cancelling active Nav2 staging goal...')
                    nav_goal_handle.cancel_goal_async()
                    goal_handle.canceled()
                    self._stop_robot()
                    result.success = False
                    result.message = "Cancelled during staging navigation"
                    return result

                feedback.distance_m = self.lidar_front
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

            nav_result = get_result_future.result()
            if nav_result.status != 4:  # GOAL_STATUS_SUCCEEDED = 4
                self.get_logger().error(f'Staging navigation failed with status code: {nav_result.status}')
                result.success = False
                result.message = "Failed to reach staging pose"
                goal_handle.abort()
                return result

            self.get_logger().info('Staging navigation complete. Transitioning to visual phases.')
            current_state = "STAGING_CORRECTION"

        state_start_t = time.perf_counter()
        search_phase = 'pan_left'
        lock_start_t = None
        loss_start_t = None
        prev_state = "SEARCHING"
        self._center_seen_start_t = None
        self._settle_start_t = None
        self._last_center_seen_t = None

        center_lock_time_s = float(self.get_parameter('center_lock_time_s').value)
        settle_time_s = float(self.get_parameter('settle_time_s').value)
        correction_timeout_s = float(self.get_parameter('staging_correction_timeout_s').value)
        correction_spin_speed = float(self.get_parameter('staging_correction_spin_speed').value)
        omega_sign_left = float(self.get_parameter('staging_correction_omega_sign_left').value)
        omega_sign_right = float(self.get_parameter('staging_correction_omega_sign_right').value)

        # Mission Loop (10Hz)
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop_robot()
                result.success = False
                result.message = "Cancelled by user"
                return result

            # Publish feedback status
            feedback.state = current_state
            feedback.distance_m = self.lidar_front
            goal_handle.publish_feedback(feedback)

            # Check if pose is valid and fresh
            pose_msg = self.dock_pose_c
            pose_fresh = (pose_msg is not None) and (time.perf_counter() - self.last_pose_t < LOSS_FULL_TIME)

            # ── State: STAGING_CORRECTION ────────────────────────────────────
            # Uses side marker visibility to rotate-in-place until the center marker is acquired.
            if current_state == "STAGING_CORRECTION":
                # If center marker is already visible, fall into SEARCHING which owns debounce/settle.
                if pose_fresh and self.marker_best_id == 0:
                    current_state = "SEARCHING"
                    state_start_t = time.perf_counter()
                    continue

                if (time.perf_counter() - state_start_t) > correction_timeout_s:
                    self.get_logger().warn('STAGING_CORRECTION timeout. Falling back to SEARCHING.')
                    current_state = "SEARCHING"
                    search_phase = 'pan_left'
                    state_start_t = time.perf_counter()
                    continue

                # Diff-drive safe: rotate in place. Use heuristic based on which side marker is seen.
                omega = 0.0
                if self.marker_best_id == 1:
                    omega = omega_sign_left * correction_spin_speed
                elif self.marker_best_id == 2:
                    omega = omega_sign_right * correction_spin_speed
                elif self.marker_best_id == 3:
                    # Both sides visible but center not: stop rotation briefly to let debounce logic catch up,
                    # then fall back to SEARCHING if center doesn't appear.
                    omega = 0.0
                else:
                    # Nothing visible: defer to SEARCHING.
                    current_state = "SEARCHING"
                    search_phase = 'pan_left'
                    state_start_t = time.perf_counter()
                    continue

                self._publish_vel(0.0, omega)
                self._publish_head(0.0, 0.0)
                time.sleep(0.1)
                continue

            # ── State: SEARCHING ──────────────────────────────────────────────
            if current_state == "SEARCHING":
                now = time.perf_counter()
                
                # Update debounce tracking if marker is visible
                if pose_fresh and self.marker_best_id == 0:
                    self._last_center_seen_t = now
                    if self._center_seen_start_t is None:
                        self._center_seen_start_t = now
                        self._settle_start_t = None
                
                # Check if we are currently debouncing (marker seen recently)
                is_debouncing = False
                if self._center_seen_start_t is None:
                    is_debouncing = False
                else:
                    # If marker drops out for more than 0.25s, abort debounce (fixes brittle resets)
                    if now - getattr(self, '_last_center_seen_t', now) > 0.25:
                        self._center_seen_start_t = None
                        self._settle_start_t = None
                        is_debouncing = False
                    else:
                        is_debouncing = True

                if is_debouncing:
                    # FIX 1: Instantly stop ALL movement the moment it's seen to prevent blur
                    self._publish_vel(0.0, 0.0)
                    # FIX 3: Keep the head frozen at the exact pan angle it was discovered at
                    self._publish_head(self._pan_angle, 0.0)

                    if (now - self._center_seen_start_t) >= center_lock_time_s:
                        if self._settle_start_t is None:
                            self._settle_start_t = now
                        elif (now - self._settle_start_t) >= settle_time_s:
                            self._center_seen_start_t = None
                            self._settle_start_t = None
                            current_state = "TRACKING"
                            state_start_t = time.perf_counter()
                    
                    time.sleep(0.1)
                    continue
                
                if time.perf_counter() - state_start_t > SEARCH_TIMEOUT:
                    result.success = False
                    result.message = "Search timeout"
                    goal_handle.abort()
                    self._stop_robot()
                    return result

                # Camera sweep & body spin
                self._publish_vel(0.0, 0.0)
                if search_phase == 'pan_left':
                    self._pan_angle += 0.05
                    if self._pan_angle >= 1.2: search_phase = 'pan_right'
                elif search_phase == 'pan_right':
                    self._pan_angle -= 0.05
                    if self._pan_angle <= -1.2: search_phase = 'recenter'
                elif search_phase == 'recenter':
                    if self._pan_angle < -0.05: self._pan_angle += 0.05
                    elif self._pan_angle > 0.05: self._pan_angle -= 0.05
                    else:
                        self._pan_angle = 0.0
                        search_phase = 'spin'
                elif search_phase == 'spin':
                    self._publish_vel(0.0, SEARCH_SPIN_SPEED)
                
                self._publish_head(self._pan_angle, 0.0)

            # ── Visual Servoing States ────────────────────────────────────────
            elif current_state in ["TRACKING", "APPROACH"]:
                if not pose_fresh:
                    self._publish_vel(0.0, 0.0)
                    if loss_start_t is None:
                        loss_start_t = time.perf_counter()
                    elif time.perf_counter() - loss_start_t > LOSS_FULL_TIME:
                        current_state = "LOST"
                        prev_state = current_state
                    continue
                
                loss_start_t = None

                # Transform pose dynamically to base_footprint using exact time stamp
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "base_footprint",
                        pose_msg.header.frame_id,
                        rclpy.time.Time.from_msg(pose_msg.header.stamp),
                        rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # Manual transform math for safety across Humble versions
                    tx = transform.transform.translation.x
                    ty = transform.transform.translation.y
                    tz = transform.transform.translation.z
                    qx = transform.transform.rotation.x
                    qy = transform.transform.rotation.y
                    qz = transform.transform.rotation.z
                    qw = transform.transform.rotation.w
                    
                    q_trans = np.array([qx, qy, qz, qw])
                    R_trans = quaternion_to_rotation_matrix(q_trans)
                    
                    px = pose_msg.pose.position.x
                    py = pose_msg.pose.position.y
                    pz = pose_msg.pose.position.z
                    p_opt = np.array([px, py, pz])
                    
                    # Target point in base footprint frame
                    p_base = np.dot(R_trans, p_opt) + np.array([tx, ty, tz])
                    
                    opx = pose_msg.pose.orientation.x
                    opy = pose_msg.pose.orientation.y
                    opz = pose_msg.pose.orientation.z
                    opw = pose_msg.pose.orientation.w
                    q_opt = np.array([opx, opy, opz, opw])
                    
                    q_base = multiply_quaternions(q_trans, q_opt)
                    q_base /= np.linalg.norm(q_base)
                    R_base = quaternion_to_rotation_matrix(q_base)
                except Exception as e:
                    self.get_logger().error(f"TF Transform failed: {e}")
                    continue

                x_d = p_base[1]  # lateral deviation (y in base_footprint)
                z_d = p_base[0]  # depth (x in base_footprint)

                # Active Camera Tracking: keep the marker in the center of the camera
                # As the base steers to face the marker, p_base[1] goes to 0, 
                # which naturally drives the camera pan back to 0.0.
                marker_angle_base = math.atan2(p_base[1], p_base[0])
                pan_error = marker_angle_base - self._pan_angle
                self._pan_angle += KP_PAN * pan_error
                self._tilt_angle = 0.0  # Lock horizontally

                # Pure Pursuit steering model (L = 0.4m lookahead)
                L = 0.4
                target_z_dock = z_d - L
                P_target_R = R_base[:, 2] * target_z_dock + p_base
                alpha = np.arctan2(P_target_R[1], P_target_R[0])

                # Control outputs
                K_omega = 0.6
                APPROACH_SPEED = 0.08  # m/s
                
                omega = K_omega * alpha
                omega = max(-0.5, min(0.5, omega))  # Steering safety clamping
                linear_v = max(0.0, APPROACH_SPEED * np.cos(alpha))

                if current_state == "TRACKING":
                    # Align body in place first
                    self._publish_vel(0.0, omega)
                    self._publish_head(self._pan_angle, self._tilt_angle)

                    if abs(alpha) < 0.1:
                        if lock_start_t is None: lock_start_t = time.perf_counter()
                        elif time.perf_counter() - lock_start_t >= LOCK_DURATION:
                            current_state = "APPROACH"
                            self.get_logger().info('Alignment locked. Moving forward.')
                    else:
                        lock_start_t = None

                elif current_state == "APPROACH":
                    # Funnel Interlock logic (depth < 0.45m)
                    if z_d < 0.45:
                        self.get_logger().info(f'Funnel Interlock engaged (z_d = {z_d:.3f}m). Locking steering straight.')
                        self._publish_vel(0.05, 0.0)
                        self._publish_head(0.0, 0.0)
                        
                        if self.lidar_front < DOCK_LIDAR_DIST:
                            current_state = "DOCKED"
                    else:
                        # Standard S-curve Pure Pursuit approach
                        self._publish_vel(linear_v, omega)
                        self._publish_head(self._pan_angle, self._tilt_angle)

            # ── State: LOST ───────────────────────────────────────────────────
            elif current_state == "LOST":
                if pose_fresh:
                    current_state = prev_state
                    continue
                
                self._publish_vel(0.0, 0.0)
                if time.perf_counter() - state_start_t > LOSS_FULL_TIME:
                    current_state = "SEARCHING"
                    search_phase = 'pan_left'
                    state_start_t = time.perf_counter()

            # ── State: DOCKED ─────────────────────────────────────────────────
            elif current_state == "DOCKED":
                self._stop_robot()
                result.success = True
                result.message = "Docking completed successfully"
                goal_handle.succeed()
                return result

            time.sleep(0.1)

    # ══════════════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _pose_cb(self, msg: PoseStamped):
        self.dock_pose_c = msg
        self.last_pose_t = time.perf_counter()

    def _marker_status_cb(self, msg: Int32MultiArray):
        # msg.data = [best_id, visible_mask]
        data = list(msg.data) if msg.data is not None else []
        if len(data) < 2:
            return
        self.marker_best_id = int(data[0])
        self.marker_visible_mask = int(data[1])

        # Throttled log on changes (helps operator validate what the coordinator sees)
        now = time.perf_counter()
        if (now - self._last_marker_log_t) < 0.5:
            return
        self._last_marker_log_t = now

        if self.marker_visible_mask == 0:
            vis = []
        else:
            vis = []
            if self.marker_visible_mask & 1: vis.append(0)
            if self.marker_visible_mask & 2: vis.append(1)
            if self.marker_visible_mask & 4: vis.append(2)

        self.get_logger().info(f'dock_vis: best={self.marker_best_id} visible={vis}')

    def _scan_cb(self, msg):
        valid = [r for r in msg.ranges if 0.05 < r < 10.0]
        self.lidar_front = min(valid) if valid else 10.0

    def _stop_robot(self):
        self._publish_vel(0.0, 0.0)
        self._publish_head(0.0, 0.0)

    def _publish_vel(self, linear, angular):
        t = Twist()
        t.linear.x, t.angular.z = linear, angular
        self.vel_pub.publish(t)
        self._last_twist = t

    def _publish_head(self, pan, tilt):
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self.head_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DockCoordinator()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

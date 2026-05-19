#!/usr/bin/env python3
"""
dock_coordinator.py

Professional ROS 2 Action Server for autonomous docking.
Handover pattern: 
1. STAGING   - Uses Nav2 to reach a fixed point in front of the dock.
2. SEARCHING - Camera sweep to find ArUco marker.
3. TRACKING  - Precise body/tilt alignment.
4. APPROACH  - Visual servoing to contact.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Bool
from yahboomcar_msgs.action import Dock

# ── Tuning constants ───────────────────────────────────────────────────────────
KP_ANGULAR       = 0.003    # Proportional gain: pixels → rad/s
DEAD_ZONE_PX     = 25       # Pixels — within this, consider "centred"
LOCK_DURATION    = 0.5      # Seconds centred before transitioning to APPROACH
APPROACH_SPEED   = 0.2      # m/s forward during APPROACH
SEARCH_SPIN_SPEED = 0.85    # rad/s body spin during SEARCHING
KP_TILT          = 0.001    # Proportional gain for vertical tracking
KP_PAN_RECENTER  = 0.1      # Speed at which head returns to center (rad/s)
SEARCH_TIMEOUT   = 30.0     # Seconds before SEARCHING gives up
LOSS_HOLD_TIME   = 0.5      # Seconds to hold last cmd before declaring LOST
LOSS_FULL_TIME   = 1.0      # Seconds lost before regressing to SEARCHING
DOCK_LIDAR_DIST  = 0.35     # m — forward lidar threshold for DOCKED
DOCK_AREA_PX     = 35000    # px² — marker area threshold for DOCKED
SCREEN_CX        = 320      # Camera width / 2
# ──────────────────────────────────────────────────────────────────────────────

class DockCoordinator(Node):

    def __init__(self):
        # Allow undeclared parameters so we can load any dock from the YAML
        super().__init__(
            'dock_coordinator',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # Use a ReentrantCallbackGroup so the action doesn't block subscribers
        self.cb_group = ReentrantCallbackGroup()

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
            Float64MultiArray, '/dock/detection', self._detection_cb, 10, callback_group=self.cb_group)
        self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10, callback_group=self.cb_group)

        # ── Shared Sensor State ───────────────────────────────────────────────
        self.marker_id   = -1.0
        self.marker_cx   = 0.0
        self.marker_cy   = 0.0
        self.marker_area = 0.0
        self.lidar_front = 10.0   # m

        # ── Internal servo state ──────────────────────────────────────────────
        self._pan_angle  = 0.0
        self._tilt_angle = 0.0
        self._last_twist = Twist()

        self.get_logger().info('Dock Action Server is ONLINE and IDLE.')

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

        # 1. Look up dock coordinates
        try:
            # We assume 'default' for now as per plan
            prefix = f'docks.{dock_id}'
            sx = self.get_parameter(f'{prefix}.staging_x').value
            sy = self.get_parameter(f'{prefix}.staging_y').value
            syaw = self.get_parameter(f'{prefix}.staging_yaw').value
        except Exception as e:
            self.get_logger().error(f'Dock ID "{dock_id}" not found in config: {e}')
            result.success = False
            result.message = f"Unknown dock ID: {dock_id}"
            goal_handle.abort()
            return result

        # 2. STAGING PHASE (Nav2)
        feedback.state = "STAGING"
        goal_handle.publish_feedback(feedback)
        
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
        nav = BasicNavigator()
        
        # Check if the navigation server is actually running
        if not nav.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Failed to connect to Nav2 (NavigateToPose action server not available)! Aborting.')
            result.success = False
            result.message = "Failed to connect to Nav2"
            goal_handle.abort()
            return result
        
        if goal_handle.request.cancel_nav:
            nav.cancelTask()
            time.sleep(0.5)

        staging_goal = PoseStamped()
        staging_goal.header.frame_id = 'map'
        staging_goal.header.stamp = self.get_clock().now().to_msg()
        staging_goal.pose.position.x = sx
        staging_goal.pose.position.y = sy
        staging_goal.pose.orientation.z = math.sin(syaw / 2.0)
        staging_goal.pose.orientation.w = math.cos(syaw / 2.0)

        self.get_logger().info(f'Navigating to staging pose: ({sx:.2f}, {sy:.2f})')
        nav.goToPose(staging_goal)

        while not nav.isTaskComplete():
            if goal_handle.is_cancel_requested:
                nav.cancelTask()
                goal_handle.canceled()
                self._stop_robot()
                result.success = False
                result.message = "Cancelled during staging"
                return result
            
            # Update distance feedback if possible
            feedback.distance_m = self.lidar_front
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)

        nav_result = nav.getResult()
        if nav_result != TaskResult.SUCCEEDED:
            self.get_logger().error(f'Staging failed with result: {nav_result}')
            result.success = False
            result.message = "Could not reach staging area"
            goal_handle.abort()
            return result

        # 3. VISUAL SERVOING PHASES
        self.get_logger().info('Staging complete. Starting visual search.')
        
        current_state = "SEARCHING"
        state_start_t = time.perf_counter()
        search_phase = 'pan_left'
        lock_start_t = None
        loss_start_t = None
        prev_state = "SEARCHING"

        # Mission Loop (10Hz)
        while rclpy.ok():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._stop_robot()
                result.success = False
                result.message = "Cancelled by user"
                return result

            # Update sensors
            feedback.state = current_state
            feedback.distance_m = self.lidar_front
            feedback.marker_area_px2 = self.marker_area
            feedback.error_x_px = self.marker_cx - SCREEN_CX
            goal_handle.publish_feedback(feedback)

            # ── State: SEARCHING ──────────────────────────────────────────────
            if current_state == "SEARCHING":
                if self.marker_id == 0.0:
                    current_state = "TRACKING"
                    state_start_t = time.perf_counter()
                    continue
                
                if time.perf_counter() - state_start_t > SEARCH_TIMEOUT:
                    result.success = False
                    result.message = "Search timeout"
                    goal_handle.abort()
                    self._stop_robot()
                    return result

                # Sweep logic
                self._publish_vel(0.0, 0.0)
                if search_phase == 'pan_left':
                    self._pan_angle += 0.1
                    if self._pan_angle >= 1.2: search_phase = 'pan_right'
                elif search_phase == 'pan_right':
                    self._pan_angle -= 0.1
                    if self._pan_angle <= -1.2: search_phase = 'recenter'
                elif search_phase == 'recenter':
                    if self._pan_angle < -0.1: self._pan_angle += 0.1
                    elif self._pan_angle > 0.1: self._pan_angle -= 0.1
                    else:
                        self._pan_angle = 0.0
                        search_phase = 'spin'
                elif search_phase == 'spin':
                    self._publish_vel(0.0, SEARCH_SPIN_SPEED)
                
                self._publish_head(self._pan_angle, 0.0)

            # ── State: TRACKING ───────────────────────────────────────────────
            elif current_state == "TRACKING":
                if self.marker_id != 0.0:
                    if loss_start_t is None: loss_start_t = time.perf_counter()
                    elif time.perf_counter() - loss_start_t > LOSS_FULL_TIME:
                        current_state = "LOST"; prev_state = "TRACKING"
                    continue
                
                loss_start_t = None
                err_x = self.marker_cx - SCREEN_CX
                ang_z = -KP_ANGULAR * err_x if abs(err_x) > DEAD_ZONE_PX else 0.0
                
                # Pan recentering + Tilt tracking
                if self._pan_angle > 0.02: self._pan_angle -= 0.02
                elif self._pan_angle < -0.02: self._pan_angle += 0.02
                
                err_y = self.marker_cy - 240
                self._tilt_angle = max(-0.8, min(0.2, self._tilt_angle - KP_TILT * err_y))

                self._publish_vel(0.0, ang_z)
                self._publish_head(self._pan_angle, self._tilt_angle)

                if abs(err_x) < DEAD_ZONE_PX and abs(self._pan_angle) < 0.1:
                    if lock_start_t is None: lock_start_t = time.perf_counter()
                    elif time.perf_counter() - lock_start_t >= LOCK_DURATION:
                        current_state = "APPROACH"
                else: lock_start_t = None

            # ── State: APPROACH ───────────────────────────────────────────────
            elif current_state == "APPROACH":
                if self.marker_id != 0.0:
                    self._publish_vel(0.0, 0.0)
                    if loss_start_t is None: loss_start_t = time.perf_counter()
                    elif time.perf_counter() - loss_start_t > LOSS_FULL_TIME:
                        current_state = "LOST"; prev_state = "APPROACH"
                    continue

                if self.lidar_front < DOCK_LIDAR_DIST and self.marker_area > DOCK_AREA_PX:
                    current_state = "DOCKED"
                    continue

                err_x = self.marker_cx - SCREEN_CX
                ang_z = -KP_ANGULAR * err_x if abs(err_x) > DEAD_ZONE_PX else 0.0
                
                if self._pan_angle > 0.02: self._pan_angle -= 0.02
                elif self._pan_angle < -0.02: self._pan_angle += 0.02
                
                err_y = self.marker_cy - 240
                self._tilt_angle = max(-0.8, min(0.2, self._tilt_angle - KP_TILT * err_y))

                self._publish_vel(APPROACH_SPEED, ang_z)
                self._publish_head(self._pan_angle, self._tilt_angle)

            # ── State: LOST ───────────────────────────────────────────────────
            elif current_state == "LOST":
                if self.marker_id == 0.0:
                    current_state = prev_state
                    continue
                
                self._publish_vel(0.0, 0.0)
                if time.perf_counter() - state_start_t > LOSS_FULL_TIME:
                    current_state = "SEARCHING"
                    search_phase = 'pan_left'

            # ── State: DOCKED ─────────────────────────────────────────────────
            elif current_state == "DOCKED":
                self._stop_robot()
                result.success = True
                result.message = "Docking complete"
                goal_handle.succeed()
                return result

            time.sleep(0.1)

    # ══════════════════════════════════════════════════════════════════════════
    # Helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _detection_cb(self, msg):
        self.marker_id, self.marker_cx, self.marker_cy, self.marker_area = msg.data

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

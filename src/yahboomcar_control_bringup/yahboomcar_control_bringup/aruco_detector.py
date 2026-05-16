"""
aruco_detector.py

Detects ArUco marker ID 0 in the robot's camera feed and publishes its
screen position and size.

Written for OpenCV 4.5.4 (ROS 2 Humble system package).
Does NOT use the new ArucoDetector class (OpenCV >= 4.7 only).

Publishes:
  /dock/detection (std_msgs/Float64MultiArray)
    data[0] = marker_id  (-1.0 if no marker detected)
    data[1] = center_x   (pixels, 0..640)
    data[2] = center_y   (pixels, 0..480)
    data[3] = area       (pixels squared, grows as robot gets closer)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np


# ArUco dictionary — must match the marker you generated
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Detection parameters — 4.5.4 uses DetectorParameters_create()
ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

# We only care about this specific marker ID (the dock)
TARGET_ID = 0

# Published when no marker is visible
NO_DETECTION = [-1.0, 0.0, 0.0, 0.0]


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()

        # Subscribe to the robot's camera
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detection results
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/dock/detection',
            10
        )

        self.get_logger().info(
            'ArUco Detector started. '
            f'Looking for DICT_4X4_50 marker ID {TARGET_ID}. '
            'OpenCV version: ' + cv2.__version__
        )

    def image_callback(self, msg: Image):
        # --- Convert ROS image to OpenCV BGR frame ---
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            return

        h, w = frame.shape[:2]
        debug = frame.copy()

        # --- Draw crosshair at screen center for reference ---
        cx_screen, cy_screen = w // 2, h // 2
        cv2.line(debug, (cx_screen - 15, cy_screen), (cx_screen + 15, cy_screen), (255, 255, 255), 1)
        cv2.line(debug, (cx_screen, cy_screen - 15), (cx_screen, cy_screen + 15), (255, 255, 255), 1)

        # --- Run ArUco detection (4.5.4 API) ---
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, ARUCO_DICT, parameters=ARUCO_PARAMS
        )

        detection_msg = Float64MultiArray()

        if ids is not None and TARGET_ID in ids.flatten():
            # Find the index of our target marker in the results list
            target_idx = list(ids.flatten()).index(TARGET_ID)
            target_corners = corners[target_idx][0]  # shape: (4, 2)

            # --- Calculate center pixel ---
            cx = float(np.mean(target_corners[:, 0]))
            cy = float(np.mean(target_corners[:, 1]))

            # --- Calculate bounding box area (proxy for distance) ---
            x_coords = target_corners[:, 0]
            y_coords = target_corners[:, 1]
            box_w = float(np.max(x_coords) - np.min(x_coords))
            box_h = float(np.max(y_coords) - np.min(y_coords))
            area = box_w * box_h

            # --- Publish detection ---
            detection_msg.data = [float(TARGET_ID), cx, cy, area]
            self.pub.publish(detection_msg)

            # --- Draw overlay on debug frame ---
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            cv2.circle(debug, (int(cx), int(cy)), 6, (0, 0, 255), -1)

            # Error from center (used by coordinator for alignment)
            error_x = cx - cx_screen
            label = f'ID:{TARGET_ID} | Err:{error_x:+.0f}px | Area:{area:.0f}'
            cv2.putText(debug, label, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        else:
            # No target marker in this frame
            detection_msg.data = NO_DETECTION
            self.pub.publish(detection_msg)
            cv2.putText(debug, 'NO MARKER', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # --- Show debug window ---
        cv2.imshow('ArUco Detector - Robot POV', debug)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        
        self.bridge = CvBridge()
        self.active = False  # Disabled by default to save CPU
        
        # Enable / Disable subscription
        self.enable_sub = self.create_subscription(
            Bool, '/color_tracker/enable', self.enable_callback, 10)
        
        # Camera Feed subscription
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
            
        # Velocity controls publisher
        self.publisher = self.create_publisher(
            Float64MultiArray, '/camera_controller/commands', 10)
            
        # Debug stream publisher (headless)
        self.debug_pub = self.create_publisher(
            Image, '/color_tracker/debug_image', 10)
        
        # Target HSV range for RED (Red wraps around 0, so we use the first half)
        self.lower_red = np.array([0, 100, 100])
        self.upper_red = np.array([10, 255, 255])
        
        # Current Head Position
        self.pan = 0.0
        self.tilt = 0.0
        
        # Control Parameters (Proportional Gains)
        self.kp_pan = 0.001   # Pixels to Radians for Pan
        self.kp_tilt = 0.001  # Pixels to Radians for Tilt
        
        self.get_logger().info("Color Tracker Node Initialized (Dormant). Waiting for activation on /color_tracker/enable...")

    def enable_callback(self, msg: Bool):
        self.active = msg.data
        self.get_logger().info(f"Color Tracker state changed. Active: {self.active}")

    def image_callback(self, msg):
        # Save CPU: Immediately return if not active
        if not self.active:
            return
            
        # 1. Convert ROS Image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return
            
        h, w, _ = frame.shape
        center_x, center_y = w // 2, h // 2
        
        # 2. Color Processing
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        
        # 3. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        target_found = False
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 500:  # Ignore tiny noise
                target_found = True
                ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
                
                # 4. Control Logic (The Tracking)
                error_x = center_x - x
                error_y = center_y - y
                
                # Update Pan (Left/Right) and Tilt (Up/Down)
                self.pan += error_x * self.kp_pan
                self.tilt += error_y * self.kp_tilt
                
                # Physical Limits
                self.pan = max(min(self.pan, 1.57), -1.57)
                self.tilt = max(min(self.tilt, 0.8), -0.8)
                
                # Send Command
                cmd = Float64MultiArray()
                cmd.data = [self.pan, self.tilt]
                self.publisher.publish(cmd)
                
                # Draw visuals
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"LOCK-ON: {int(area)}px", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 5. Add Reference Crosshairs to Frame
        cv2.line(frame, (center_x - 10, center_y), (center_x + 10, center_y), (255, 255, 255), 1)
        cv2.line(frame, (center_x, center_y - 10), (center_x, center_y + 10), (255, 255, 255), 1)
        
        # 6. Publish Debug Frame Headless
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.debug_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

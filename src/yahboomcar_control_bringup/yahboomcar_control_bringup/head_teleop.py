import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import termios
import tty

msg = """
Control Your Robot's Head! (Run this in a SEPARATE terminal)
---------------------------
Moving around:
        t
   f    g    h

t : Tilt Up
g : Tilt Down
f : Pan Left
h : Pan Right

Space : Reset to center
CTRL-C to quit
"""

move_bindings = {
    't': (0.0, 0.1),
    'g': (0.0, -0.1),
    'f': (0.1, 0.0),
    'h': (-0.1, 0.0),
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

class HeadTeleop(Node):
    def __init__(self):
        super().__init__('head_teleop')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/camera_controller/commands', 10)
        self.pan = 0.0
        self.tilt = 0.0
        self.get_logger().info("Head Teleop Started. Use i,j,k,l to move. Space to reset.")

    def update_position(self, delta_pan, delta_tilt):
        self.pan += delta_pan
        self.tilt += delta_tilt
        
        # Keep within reasonable physical limits (~90 degrees)
        self.pan = max(min(self.pan, 1.57), -1.57)
        self.tilt = max(min(self.tilt, 0.8), -0.8)
        
        msg = Float64MultiArray()
        msg.data = [self.pan, self.tilt]
        self.publisher_.publish(msg)

    def reset(self):
        self.pan = 0.0
        self.tilt = 0.0
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self.publisher_.publish(msg)

def main():
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = HeadTeleop()

    print(msg)

    try:
        while True:
            key = get_key(settings)
            if key in move_bindings.keys():
                pan_d, tilt_d = move_bindings[key]
                node.update_position(pan_d, tilt_d)
                print(f"\rPan: {node.pan:.2f} | Tilt: {node.tilt:.2f}    ", end="")
            elif key == ' ':
                node.reset()
                print(f"\rReset to Center                ", end="")
            elif key == '\x03':  # CTRL-C
                break
    except Exception as e:
        print(e)
    finally:
        node.reset()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

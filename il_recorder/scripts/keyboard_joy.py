#!/usr/bin/env python3
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

"""

Script to simulate joystick inputs via keyboard for controlling il_recorder.

Note that this is only for start/stop/cancel recording, not for moving the robot, but is useful
for debugging, or if you have some other method of controlling the robot separate from joystick.

Usage: ros2 run il_recorder keyboard_joy

"""


class KeyboardJoy(Node):
    def __init__(self):
        super().__init__("keyboard_joy")
        self.pub = self.create_publisher(Joy, "/joy", 10)

        print("\n-------------------------------------------")
        print("FAKE JOYSTICK CONTROLLER")
        print("-------------------------------------------")
        print(" [SPACE] : Toggle Record (Button 7)")
        print(" [x]     : Cancel/Discard (Button 8)")
        print(" [q]     : Quit")
        print("-------------------------------------------\n")

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def send_button_tap(self, button_index):
        """
        Simulates a press and release of a specific button index.
        """
        # 1. Create the "Pressed" message
        msg_down = Joy()
        msg_down.header.stamp = self.get_clock().now().to_msg()
        # Create array large enough (your logic checks index 7 and 8)
        msg_down.buttons = [0] * 12
        msg_down.axes = [0.0] * 6

        # PRESS
        msg_down.buttons[button_index] = 1
        self.pub.publish(msg_down)

        action = "START/STOP" if button_index == 7 else "CANCEL"
        self.get_logger().info(f"Sent: Button {button_index} ({action})")

        # 2. Short sleep to ensure the recorder sees the 'down' state
        time.sleep(0.1)

        # 3. Create the "Released" message
        msg_up = Joy()
        msg_up.header.stamp = self.get_clock().now().to_msg()
        msg_up.buttons = [0] * 12
        msg_up.axes = [0.0] * 6

        # RELEASE
        msg_up.buttons[button_index] = 0
        self.pub.publish(msg_up)

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == " ":
                self.send_button_tap(7)

            elif key == "x":
                self.send_button_tap(8)

            elif key == "q":
                break
            elif key == "\x03":  # Ctrl+C
                break


def main():
    rclpy.init()
    node = KeyboardJoy()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

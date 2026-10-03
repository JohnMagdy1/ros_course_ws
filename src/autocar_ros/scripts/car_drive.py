#!/usr/bin/env python3
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

import argparse

# Vehicle-specific control parameters
LINEAR_INCREMENT = 1       # m/s per keypress
ANGULAR_INCREMENT = 3      # rad/s per keypress
MAX_LINEAR_SPEED = 10.0    # m/s max forward speed
MAX_ANGULAR_SPEED = 60.0   # rad/s max turning speed

msg = """
===================================================
VEHICLE TELEOPERATION CONTROL
===================================================

CONTROLS:
    W/S      : Increase/Decrease Forward Speed
    A/D      : Increase/Decrease Turning (Steering)
    SPACE    : Emergency Stop

    CTRL+C   : Exit

CURRENT CONTROLS:
    Forward Speed: {linear:.2f} m/s
    Turning Rate: {angular:.2f} rad/s
    Mode: {mode}
"""


class VehicleTeleop:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())          # raw input, set once

        # Keep raw input, restore normal output processing
        attrs = termios.tcgetattr(sys.stdin)
        attrs[1] |= termios.OPOST | termios.ONLCR
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, attrs)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.emergency_stop = False
        self.reset_angular = False
        self.smoothing_factor = 1

        rospy.init_node('vehicle_teleop_keyboard')
        parser = argparse.ArgumentParser()
        parser.add_argument('-ns', '--namespace', type=str, default="")
        args, _ = parser.parse_known_args()
        ns = args.namespace
        topic_name = ns + "/cmd_vel" if ns else "cmd_vel"

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=1)
        self.rate = rospy.Rate(20)

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if rlist else ''

    def smooth_control(self, target, current):
        return current + self.smoothing_factor * (target - current)

    def apply_limits(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def update_display(self):
        """Clear terminal and redraw the control panel."""
        sys.stdout.write("\033[H\033[J")   # cursor home + clear
        mode = "EMERGENCY STOP" if self.emergency_stop else "FORWARD"
        sys.stdout.write(msg.format(
            linear=self.current_linear,
            angular=abs(self.current_angular),
            mode=mode
        ))
        sys.stdout.write("\n")
        sys.stdout.flush()

    def emergency_stop_procedure(self):
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.emergency_stop = True

    def handle_key(self, key):
        """Process a keypress. Returns False to exit the loop."""
        if key == '\x03':                       # Ctrl+C
            return False

        # Emergency stop override
        if self.emergency_stop:
            if key == ' ':
                self.emergency_stop = False
            else:
                self.target_linear = 0.0
                self.target_angular = 0.0

        # Control mappings
        elif key == 'w':
            if not self.reset_angular:
                self.target_linear += LINEAR_INCREMENT
            self.target_angular = 0
            self.reset_angular = False

        elif key == 's':
            if not self.reset_angular:
                self.target_linear -= LINEAR_INCREMENT
            self.target_angular = 0
            self.reset_angular = False

        elif key == 'a' and self.current_linear != 0:
            self.target_angular += ANGULAR_INCREMENT
            self.reset_angular = True

        elif key == 'd' and self.current_linear != 0:
            self.target_angular -= ANGULAR_INCREMENT
            self.reset_angular = True

        elif key == ' ':
            self.emergency_stop_procedure()

        return True

    def run(self):
        try:
            sys.stdout.write("Starting Vehicle Teleoperation...\n")
            sys.stdout.flush()
            rospy.sleep(1)

            self.update_display()               # initial draw

            while not rospy.is_shutdown():
                key = self.get_key()

                if key:
                    if not self.handle_key(key):
                        break

                # Limits (run every loop)
                self.target_linear = self.apply_limits(
                    self.target_linear, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
                self.target_angular = self.apply_limits(
                    self.target_angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

                # Smoothing (run every loop)
                self.current_linear = self.smooth_control(
                    self.target_linear, self.current_linear)
                self.current_angular = self.smooth_control(
                    self.target_angular, self.current_angular)

                # Publish (run every loop)
                twist = Twist()
                twist.linear.x = self.current_linear
                twist.angular.z = self.current_angular
                self.pub.publish(twist)
                self.update_display()       # redraw only on keypress

                self.rate.sleep()

        except Exception as e:
            sys.stdout.write(f"Error: {e}\n")
            sys.stdout.flush()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            twist = Twist()
            self.pub.publish(twist)
            sys.stdout.write("\nVehicle stopped. Exiting...\n")
            sys.stdout.flush()


if __name__ == '__main__':
    try:
        teleop = VehicleTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

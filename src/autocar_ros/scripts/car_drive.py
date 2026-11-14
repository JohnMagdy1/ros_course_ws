#!/usr/bin/env python3
import rospy
import sys, select, termios, tty
import math
from std_msgs.msg import Bool, Float32MultiArray
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
    E        : Toggle AEB (local override)
    SPACE    : Emergency Stop

    CTRL+C   : Exit

STATUS:
    DRIVING Speed : {linear:.2f} m/s
    Turning Rate  : {angular:.2f} rad/s
    Mode          : {mode}
    AEB           : {aeb}   d={dist}  v_r={vr}  ttc={ttc}
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

        # AEB state (fed by aeb_node)
        self.d = None
        self.vr = None
        self.ttc = None
        self.aeb_brake_signal = False
        self.aeb_enabled = True
        self.braking = False

        rospy.init_node('vehicle_teleop_keyboard')
        parser = argparse.ArgumentParser()
        parser.add_argument('-ns', '--namespace', type=str, default="")
        args, _ = parser.parse_known_args()
        ns = args.namespace
        cmd_topic = ns + "/cmd_vel" if ns else "cmd_vel"
        status_topic = ns + "/radar/status" if ns else "/radar/status"
        brake_topic  = ns + "/aeb/brake"  if ns else "/aeb/brake"

        self.pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(status_topic, Float32MultiArray, self.status_cb)
        rospy.Subscriber(brake_topic,  Bool,               self.brake_cb)
        self.rate = rospy.Rate(20)

        rospy.loginfo("Teleop -> %s   (AEB: %s, %s)", cmd_topic, status_topic, brake_topic)

    # ---- aeb_node inputs ----
    def status_cb(self, m):
        if len(m.data) < 3:
            return
        d, v, ttc = m.data[0], m.data[1], m.data[2]
        self.d   = None if math.isinf(d)   else float(d)
        self.vr  = None if self.d is None  else float(v)
        self.ttc = None if math.isinf(ttc) else float(ttc)

    def brake_cb(self, m):
        self.aeb_brake_signal = bool(m.data)

    # ---- decide this-frame brake (local toggle AND remote signal) ----
    def _update_brake(self):
        self.braking = self.aeb_enabled and self.aeb_brake_signal

    def get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        return sys.stdin.read(1) if rlist else ''

    def smooth_control(self, target, current):
        return current + self.smoothing_factor * (target - current)

    def apply_limits(self, value, min_val, max_val):
        return max(min_val, min(value, max_val))

    def update_display(self):
        sys.stdout.write("\033[H\033[J")
        if self.emergency_stop:
            mode = "EMERGENCY STOP"
        elif self.braking:
            mode = "AEB BRAKING"
        elif not self.aeb_enabled:
            mode = "DRIVING (AEB OFF)"
        else:
            mode = "DRIVING"
        aeb   = "ON " if self.aeb_enabled else "OFF"
        dist  = f"{self.d:.1f}m"    if self.d   is not None else "--"
        vr    = f"{self.vr:.2f}m/s" if self.vr  is not None else "--"
        ttc_s = f"{self.ttc:.1f}s"  if self.ttc is not None else "--"
        sys.stdout.write(msg.format(
            linear=self.current_linear,
            angular=abs(self.current_angular),
            mode=mode, aeb=aeb, dist=dist, vr=vr, ttc=ttc_s
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

        # E toggles AEB
        if key == 'e':
            self.aeb_enabled = not self.aeb_enabled
            if not self.aeb_enabled:
                self.braking = False

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

                self._update_brake()  # update braking state based on AEB signal

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

                if self.emergency_stop or self.braking:
                    # hard e-stop: clear targets, driver must press W to resume
                    self.target_linear   = 0.0
                    self.target_angular  = 0.0
                    self.current_linear  = 0.0
                    self.current_angular = 0.0
                    self.pub.publish(Twist())
                else:
                    twist = Twist()
                    twist.linear.x  = self.current_linear
                    twist.angular.z = self.current_angular
                    self.pub.publish(twist)

                self.update_display()
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

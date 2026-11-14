#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from ainstein_radar_msgs.msg import RadarTargetArray


class RangeTracker:
    """1D constant-velocity Kalman filter on a single target's range.

    State: x = [range, range_rate]
      - range      (m)
      - range_rate (m/s) — negative = closing, positive = receding

    The radar plugin only publishes range; speed is always 0.0. This filter
    infers speed by tracking how range evolves across frames: if the measured
    range is consistently shorter than the filter predicted, the velocity
    estimate is nudged negative (closing). A single measurement cannot do this
    — velocity only emerges from the pattern across 3-5 frames.

    One tracker instance exists per radar target_id (i.e. per ray), so each
    ray maintains its own independent estimate.
    """
    def __init__(self, r0, t0):
        self.x = np.array([r0, 0.0])
        self.P = np.diag([1.0, 10.0])
        self.t = t0

    def update(self, r_meas, t_now):
        dt = t_now - self.t
        if dt <= 1e-5:
            return self.x[0], self.x[1]
        F = np.array([[1.0, dt], [0.0, 1.0]])
        Q = np.diag([1e-3 * dt, 5e-1 * dt])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        H = np.array([[1.0, 0.0]])
        R = np.array([[0.05 ** 2]])
        y = r_meas - (H @ self.x)[0]
        S = H @ self.P @ H.T + R
        K = (self.P @ H.T) @ np.linalg.inv(S)
        self.x = self.x + (K.flatten() * y)
        self.P = (np.eye(2) - K @ H) @ self.P
        self.t = t_now
        return self.x[0], self.x[1]


class RadarReader:
    def __init__(self):

        # ---- internal state ----
        self.trackers      = {}
        self.d_offset      = (3.8/2) - 0.7
        
        # ---- io ----
        rospy.Subscriber('/radar/targets', RadarTargetArray, self.cb)
        self.status_pub = rospy.Publisher('/radar/status', Float32MultiArray, queue_size=1)

    # ---- helpers ----
    def _ttc(self, d, v):
        if d is None or v is None:
            return None
        if v >= 0:
            return None
        return d / (-v)
    def cb(self, msg):
        now = msg.header.stamp.to_sec()
        seen = set()

        for t in msg.targets:
            tid = t.target_id
            seen.add(tid)
            if tid not in self.trackers:
                self.trackers[tid] = RangeTracker(t.range, now)
                t.speed = 0.0
            else:
                t.range, t.speed = self.trackers[tid].update(t.range, now)

        self.trackers = {k: v for k, v in self.trackers.items() if k in seen}

        status = Float32MultiArray()

        # ---- pick closest valid target ----
        valid = [t for t in msg.targets if t.range >= 0.3]
        if valid:
            closest = min(valid, key=lambda t: t.range)
            d = closest.range - self.d_offset
            v = closest.speed
        else:
            d, v = None, None

        ttc = self._ttc(d, v)
        # ---- publish status array [d, v, ttc] ----

        status.data = [
            float(d)   if d   is not None else float('inf'),
            float(v)   if v   is not None else 0.0,
            float(ttc) if ttc is not None else float('inf'),
        ]
        self.status_pub.publish(status)

        if d and v:
            rospy.loginfo("d = %.2f m   v_r = %.2f m/s", d, v)


if __name__ == '__main__':
    rospy.init_node('radar_reader')
    RadarReader()
    rospy.spin()
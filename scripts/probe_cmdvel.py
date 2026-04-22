"""Short rotate/backward/forward cycles to see if failure is position-dependent."""
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class Probe(Node):
    def __init__(self):
        super().__init__("probe_cmdvel")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.j = None
        self.o = None

    def _on_joints(self, m): self.j = m
    def _on_odom(self, m): self.o = m

    def pub_cmd(self, lx, az):
        t = Twist(); t.linear.x = float(lx); t.angular.z = float(az); self.pub.publish(t)

    def snap(self, tag):
        l, r = None, None
        if self.j:
            v = dict(zip(self.j.name, self.j.velocity))
            l, r = v.get("drivewhl_l_joint"), v.get("drivewhl_r_joint")
        if self.o:
            p = self.o.pose.pose.position
            lin = self.o.twist.twist.linear
            ang = self.o.twist.twist.angular
            q = self.o.pose.pose.orientation
            import math
            # yaw from quaternion
            siny = 2.0 * (q.w*q.z + q.x*q.y); cosy = 1 - 2*(q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny, cosy)
            print(f"[{tag}] pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) yaw={math.degrees(yaw):+.2f}deg "
                  f"lin=({lin.x:+.3f},{lin.y:+.3f}) ang_z={ang.z:+.3f} "
                  f"wheels=(L={l:+.2f},R={r:+.2f})rad/s" if l is not None and r is not None
                  else f"[{tag}] pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f}) yaw={math.degrees(yaw):+.2f}deg "
                       f"wheels=<none>")


def phase(node, tag, lx, az, dur, snap_every=0.3):
    print(f"\n--- {tag}: cmd=({lx}, {az}) for {dur}s ---")
    t_end = time.time() + dur
    next_snap = time.time()
    next_pub = time.time()
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now >= next_pub:
            node.pub_cmd(lx, az); next_pub = now + 0.1
        if now >= next_snap:
            node.snap(tag); next_snap = now + snap_every


def main():
    rclpy.init()
    n = Probe()
    end = time.time() + 1.0
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.05)

    # First: rotate in place to reorient. Then try forward+backward from new yaw.
    phase(n, "ROT_CCW",  lx=0.0, az=+0.5, dur=3.0)
    phase(n, "STOP1",    lx=0.0, az=0.0, dur=1.0)
    phase(n, "FWD",      lx=+0.2, az=0.0, dur=2.0)
    phase(n, "STOP2",    lx=0.0, az=0.0, dur=1.0)
    phase(n, "BWD",      lx=-0.2, az=0.0, dur=2.0)
    phase(n, "STOP3",    lx=0.0, az=0.0, dur=1.0)
    # Now try rotate in-place in opposite direction:
    phase(n, "ROT_CW",   lx=0.0, az=-0.5, dur=3.0)
    phase(n, "FINAL",    lx=0.0, az=0.0, dur=1.0)

    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()

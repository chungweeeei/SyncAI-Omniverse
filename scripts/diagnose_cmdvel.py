"""Drive /cmd_vel with a forward/backward/rotate sequence and print the
live /joint_states and /odom response. Exposes whether the fault is in
the message pipeline (DifferentialController clamping), the drive gains,
or the physics contact.

Run inside the isaac-sim container with an Isaac Sim sim already running
and publishing the usual topics:

    docker exec -it syncai-simulation bash
    export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib
    export PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/rclpy
    /isaac-sim/python.sh /workspace/scripts/diagnose_cmdvel.py
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class CmdVelDiagnostic(Node):
    def __init__(self):
        super().__init__("diagnose_cmdvel")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.latest_joints = None
        self.latest_odom = None

    def _on_joints(self, msg: JointState) -> None:
        self.latest_joints = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def publish(self, lin_x: float, ang_z: float) -> None:
        t = Twist()
        t.linear.x = float(lin_x)
        t.angular.z = float(ang_z)
        self.pub.publish(t)

    def snapshot(self, label: str) -> None:
        js = self.latest_joints
        od = self.latest_odom
        if js is None:
            joint_line = "joint_states=<none yet>"
        else:
            # Zip name/velocity and only print the two drive joints.
            vels = dict(zip(js.name, js.velocity))
            l = vels.get("drivewhl_l_joint")
            r = vels.get("drivewhl_r_joint")
            joint_line = (
                f"wheels=(L={l:+.3f},R={r:+.3f})rad/s"
                if l is not None and r is not None
                else f"joint_states names={list(vels.keys())}"
            )
        if od is None:
            odom_line = "odom=<none yet>"
        else:
            v = od.twist.twist.linear
            w = od.twist.twist.angular
            p = od.pose.pose.position
            odom_line = (
                f"pos=({p.x:+.3f},{p.y:+.3f},{p.z:+.3f})  "
                f"lin=({v.x:+.3f},{v.y:+.3f},{v.z:+.3f})  ang_z={w.z:+.3f}"
            )
        print(f"[{label}] {joint_line}  {odom_line}")


def _run_phase(node: CmdVelDiagnostic, label: str, lin: float, ang: float,
               duration: float = 2.5, snap_interval: float = 0.5) -> None:
    print(f"\n===== PHASE: {label}  cmd_vel=(linear.x={lin}, angular.z={ang}) =====")
    end = time.time() + duration
    next_snap = time.time()
    next_pub = time.time()
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now >= next_pub:
            node.publish(lin, ang)
            next_pub = now + 0.1   # 10 Hz
        if now >= next_snap:
            node.snapshot(label)
            next_snap = now + snap_interval


def main() -> None:
    rclpy.init()
    node = CmdVelDiagnostic()

    # Allow publishers/subscribers to discover peers before the first phase.
    print("Waiting 1.0 s for discovery...")
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    try:
        _run_phase(node, "IDLE",     lin=0.0,  ang=0.0)
        _run_phase(node, "FORWARD",  lin=0.3,  ang=0.0)
        _run_phase(node, "STOP",     lin=0.0,  ang=0.0, duration=1.5)
        _run_phase(node, "BACKWARD", lin=-0.3, ang=0.0)
        _run_phase(node, "STOP",     lin=0.0,  ang=0.0, duration=1.5)
        _run_phase(node, "ROTATE+",  lin=0.0,  ang=0.8)
        _run_phase(node, "STOP",     lin=0.0,  ang=0.0, duration=1.5)
        _run_phase(node, "ROTATE-",  lin=0.0,  ang=-0.8)
        _run_phase(node, "FINAL",    lin=0.0,  ang=0.0, duration=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

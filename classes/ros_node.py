"""ROS 2 integration – subscriber node and background spin thread.

Subscribes to:
  - legs_feedback      (Float32MultiArray)  –  12-DOF leg joints in degrees
  - upperbody_feedback (Float32MultiArray)  –  7-DOF upper-body joints in degrees

Publishes to:
  - status_command     (Int16MultiArray)   –  [COLLISION_FLAG, 1|0]
"""
import math
import threading

from .shared_state import SharedState

# ---------------------------------------------------------------------------
# Optional ROS 2 import
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32MultiArray, Int16MultiArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARN] rclpy not found – ROS Control mode will be disabled.")

# ---------------------------------------------------------------------------
# Servo ID → actuator key mapping  (robot.py appends "_p" to actuator names)
# ---------------------------------------------------------------------------
ROS_ID_TO_ACTUATOR: dict[int, str] = {
    16: "RHip_yaw_p",
     6: "RHip_roll_p",
     7: "RHip_pitch_p",
     8: "RKnee_pitch_p",
     9: "RAnkle_pitch_p",
    10: "RAnkle_roll_p",
    17: "LHip_yaw_p",
    11: "LHip_roll_p",
    12: "LHip_pitch_p",
    13: "LKnee_pitch_p",
    14: "LAnkle_pitch_p",
    15: "LAnkle_roll_p",
    19: "core_p",
     0: "RShoulder_pitch_p",
     1: "RShoulder_roll_p",
     2: "RElbow_pitch_p",
     3: "LShoulder_pitch_p",
     4: "LShoulder_roll_p",
    18: "LElbow_pitch_p",
}

LEGS_IDS      = [16,  6,  7,  8, 10,  9, 17, 11, 12, 13, 15, 14]
UPPERBODY_IDS = [ 0,  1,  2,  3,  4, 18, 19]

# Collision flag index published on status_command topic
COLLISION_DETECTION_FLAG = 9
STATUS_COMMAND_TOPIC     = "status_command"


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------
class NubiRosNode(Node if ROS_AVAILABLE else object):
    """Subscribes to joint feedback topics and writes angles into SharedState.
    Also provides publish_collision_status() for the LiveCollisionMonitor.
    """

    def __init__(self, shared_state: SharedState):
        if not ROS_AVAILABLE:
            return
        super().__init__("nubi_simulation")
        self.shared = shared_state
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Float32MultiArray, "legs_feedback",
                                 self._legs_cb, qos)
        self.create_subscription(Float32MultiArray, "upperbody_feedback",
                                 self._upperbody_cb, qos)

        self._status_pub = self.create_publisher(
            Int16MultiArray, STATUS_COMMAND_TOPIC,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    # ------------------------------------------------------------------
    def publish_collision_status(self, colliding: bool) -> None:
        """Publish collision state to the status_command topic."""
        if not ROS_AVAILABLE:
            return
        msg = Int16MultiArray()
        msg.data = [COLLISION_DETECTION_FLAG, 1 if colliding else 0]
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    def _apply(self, ids_array: list[int], angles) -> None:
        overrides = {}
        for ros_id, angle in zip(ids_array, angles):
            if angle > 900:          # sentinel for invalid
                continue
            act_name = ROS_ID_TO_ACTUATOR.get(ros_id)
            if act_name is None:
                continue
            overrides[act_name] = math.radians(float(angle))

        with self.shared.lock:
            if self.shared.mode == "ros":
                self.shared.ctrl_overrides.update(overrides)

    def _legs_cb(self, msg: "Float32MultiArray") -> None:
        self._apply(LEGS_IDS, msg.data)

    def _upperbody_cb(self, msg: "Float32MultiArray") -> None:
        self._apply(UPPERBODY_IDS, msg.data)


# ---------------------------------------------------------------------------
# Background spin helper
# ---------------------------------------------------------------------------
def start_ros_thread(shared_state: SharedState) -> "NubiRosNode | None":
    """Initialise rclpy and spin the ROS node in a daemon thread."""
    if not ROS_AVAILABLE:
        return None
    rclpy.init()
    node = NubiRosNode(shared_state)

    def _spin():
        try:
            rclpy.spin(node)
        except Exception:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

    t = threading.Thread(target=_spin, daemon=True)
    t.start()
    return node

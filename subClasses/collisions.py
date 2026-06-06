#!/usr/bin/env python3
import sys
import time
import math
import threading
import queue
import rclpy
import rclpy.node
import mujoco
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from pathlib import Path
from PyQt6.QtCore import QObject, pyqtSignal

try:
    from settings.settings import *
except ModuleNotFoundError:
    parent_dir = str(Path(__file__).resolve().parent.parent)
    print("ran directly")
    # 2. Add it to Python's system path
    sys.path.append(parent_dir)
    from settings.settings import *

# ─────────────────────────────────────────────────────────────────
#  PyQt6 Signals (Thread-safe bridge between ROS and GUI)
# ─────────────────────────────────────────────────────────────────
class VerifierSignals(QObject):
    batch_started = pyqtSignal(int)          # Total items in batch
    progress_updated = pyqtSignal(int, int)  # Current index, Total
    invalid_detected = pyqtSignal(int, list) # Index, Angles
    needs_confirmation = pyqtSignal()        # Triggers the Proceed button
    publishing_status = pyqtSignal(str)      # Updates the publish label

# ─────────────────────────────────────────────────────────────────
#  MuJoCo collision checker
# ─────────────────────────────────────────────────────────────────
_model:      mujoco.MjModel | None = None
_check_data: mujoco.MjData  | None = None
_mj_lock = threading.Lock()

_MODEL_PATH = str(Path(__file__).resolve().parent.parent / "documents" / "nubi.xml")


def _init_mujoco(model_path: str) -> None:
    global _model, _check_data
    _model      = mujoco.MjModel.from_xml_path(model_path)
    _check_data = mujoco.MjData(_model)

def has_self_collision(q: list) -> bool:
    """q must be in degrees (servo command units). Converted to radians internally."""
    with _mj_lock:
        _check_data.qpos[-len(q):] = [math.radians(a) for a in q]
        mujoco.mj_kinematics(_model, _check_data)
        mujoco.mj_collision(_model, _check_data)

        for i in range(_check_data.ncon):
            g1 = _check_data.contact[i].geom1
            g2 = _check_data.contact[i].geom2
            if _model.geom_contype[g1] == 2 and _model.geom_contype[g2] == 2:
                return True
    return False

# ─────────────────────────────────────────────────────────────────
#  ROS 2 node
# ─────────────────────────────────────────────────────────────────
class CommandVerifier(rclpy.node.Node):
    def __init__(self, signals: VerifierSignals = None):
        super().__init__("command_verifier")
        self.signals = signals

        # Event flag to block/unblock batch processing from the GUI
        self.proceed_event = threading.Event()
        self.proceed_event.set() 

        # ── declare + read parameters ──────────────────────
        self.declare_parameter("batch_window", 0.05)
        self.declare_parameter("send_delay",   0.0)
        model_path = _MODEL_PATH

        self._batch_window: float = self.get_parameter("batch_window").value
        self._send_delay:   float = self.get_parameter("send_delay").value

        # ── load MuJoCo ────────────────────────────────────
        _init_mujoco(model_path)
        self.get_logger().info(f"MuJoCo model loaded: '{model_path}'")
        self.get_logger().info(f"Batch window: {self._batch_window}s | Send delay: {self._send_delay}s")

        # ── batch accumulator ──────────────────────────────
        self._buffer: list[list]              = []
        self._buffer_lock                     = threading.Lock()
        self._batch_timer: threading.Timer | None = None

        self._send_queue: queue.Queue = queue.Queue()

        # ── publishers / subscribers ───────────────────────
        # Verified leg angles → STM legs topic
        self._legs_cmd_pub = self.create_publisher(LEGS_MSG_TYPE, LEGS_PUB_TOPIC, LEGS_PUB_QOS)
        # Verified upper-body HS angles → STM upperbody topic
        self._upperbody_cmd_pub = self.create_publisher(UPPERBODY_MSG_TYPE, UPPERBODY_PUB_TOPIC, UPPERBODY_PUB_QOS)
        # Collision flag → status_command topic (index COLLISION_DETECTION_FLAG)
        self._status_cmd_pub = self.create_publisher(STATUS_MSG_TYPE, STATUS_COMMAND_TOPIC, STATUS_PUB_QOS)

        self.create_subscription(LEGS_MSG_TYPE, LEGS_SUB_TOPIC,       self._legs_feedback_cb,  LEGS_SUB_QOS)
        self.create_subscription(UPPERBODY_MSG_TYPE, UPPERBODY_SUB_TOPIC,   self._upper_feedback_cb, UPPERBODY_SUB_QOS)
        self.create_subscription(COLLISION_VALIDATION_MSG_TYPE, COLLISION_VALIDATION_TOPIC,  self._verification_cb,   COLLISION_VALIDATION_PUB_QOS)

        self.get_logger().info(f"Subscribed to feedback: '{LEGS_SUB_TOPIC}' | '{UPPERBODY_SUB_TOPIC}'")
        self.get_logger().info(f"Subscribed to verification commands: '{COLLISION_VALIDATION_TOPIC}'")
        self.get_logger().info(f"Publishing verified commands → '{LEGS_PUB_TOPIC}' | '{UPPERBODY_PUB_TOPIC}'")
        self.get_logger().info(f"Publishing collision flag → '{STATUS_COMMAND_TOPIC}' (index {COLLISION_DETECTION_FLAG})")

        # Last-known feedback angles for live collision monitoring
        # Legs in LEGS_HS_CMD_IDS order (12), upper in UPPERBODY_HS_CMD_IDS order (7)
        self._last_legs_angles:  list | None = None
        self._last_upper_angles: list | None = None
        self._feedback_state_lock = threading.Lock()
        # Throttle live collision checks to 20 Hz max
        self._last_live_check_time = 0.0
        self._live_check_interval  = 0.05  # seconds (1/20 Hz)

        # ── background sender thread ───────────────────────
        self._sender = threading.Thread(target=self._send_worker, daemon=True)
        self._sender.start()

    def _legs_feedback_cb(self, msg: Float32MultiArray) -> None:
        with self._feedback_state_lock:
            self._last_legs_angles = list(msg.data)

        #for collision check, all angles must be read
        if any(v >= 1000 for v in msg.data):  # Sentinel value indicating missing/unpowered servo            
            #self.get_logger().info("[FEEDBACK] Skipping live collision check — legs feedback contains sentinel value(s)")
            return
        self.get_logger().info(f"[FEEDBACK] legs ({len(msg.data)} values): {[round(v,2) for v in msg.data]}")
        
        self._check_live_collision()

    def _upper_feedback_cb(self, msg: Float32MultiArray) -> None:
        with self._feedback_state_lock:
            self._last_upper_angles = list(msg.data)
            #for collision check, all angles must be read
        if any(v >= 1000 for v in msg.data):  # Sentinel value indicating missing/unpowered servo            
            #self.get_logger().info("[FEEDBACK] Skipping live collision check — UpperBody feedback contains sentinel value(s)")
            return
        self.get_logger().info(f"[FEEDBACK] upper ({len(msg.data)} values): {[round(v,2) for v in msg.data]}")
        self._check_live_collision()

    def _check_live_collision(self) -> None:
        """Build mujoco-ordered angle array from latest legs + upper feedback and check collision.
        Only runs when both feedback arrays are available. STD servos are not in mujoco.
        Throttled to self._live_check_interval (default 20 Hz) to avoid overloading MuJoCo."""
        now = time.monotonic()
        if now - self._last_live_check_time < self._live_check_interval:
            return
        self._last_live_check_time = now
        with self._feedback_state_lock:
            if self._last_legs_angles is None or self._last_upper_angles is None:
                self.get_logger().info("[LIVE CHECK] Skipped — waiting for both legs and upper feedback")
                return
            legs   = list(self._last_legs_angles)
            upper  = list(self._last_upper_angles)

        # Build id → angle from feedback (filter out sentinel values ≥ 1000)
        id_to_angle: dict[int, float] = {}
        for sid, angle in zip(LEGS_HS_CMD_IDS, legs):
            if abs(angle) < 1000:
                id_to_angle[sid] = angle
        for sid, angle in zip(UPPERBODY_HS_CMD_IDS, upper):
            if abs(angle) < 1000:
                id_to_angle[sid] = angle

        # Build mujoco-ordered array — skip check if any servo is missing / unpowered
        mujoco_q = []
        for sid in MUJOCO_SAFETY_ID_ORDER:
            if sid not in id_to_angle:
                self.get_logger().info(f"[LIVE CHECK] Skipped — servo id={sid} missing or unpowered (sentinel)")
                return  # incomplete — don't check
            mujoco_q.append(id_to_angle[sid])

        self.get_logger().info(f"[LIVE CHECK] mujoco_q ({len(mujoco_q)} joints): {[round(v,2) for v in mujoco_q]}")
        colliding = has_self_collision(mujoco_q)
        if colliding:
            self.get_logger().warn("⚠ SELF-COLLISION detected in live feedback!")
        else:
            self.get_logger().info("[LIVE CHECK] No collision detected")
        # Publish collision flag on status_command topic (index COLLISION_DETECTION_FLAG)
        flag_msg = Int16MultiArray()
        flag_msg.data = [COLLISION_DETECTION_FLAG, 1 if colliding else 0]
        self._status_cmd_pub.publish(flag_msg)

    def _verification_cb(self, msg: Float32MultiArray) -> None:
        angles = list(msg.data)
        self.get_logger().info(f"[VERIFY IN] Received {len(angles)}-element command: mujoco={[round(v,2) for v in angles[:19]]} | std={[round(v,2) for v in angles[19:23]]} | playtime={angles[-1] if len(angles)>19 else '?'}ms")
        with self._buffer_lock:
            self._buffer.append(angles)
            if self._batch_timer is not None:
                self._batch_timer.cancel()
            self._batch_timer = threading.Timer(self._batch_window, self._process_batch)
            self._batch_timer.start()

    def _process_batch(self) -> None:
        with self._buffer_lock:
            batch = self._buffer[:]
            self._buffer.clear()
            self._batch_timer = None

        if not batch:
            return

        self.get_logger().info(f"[BATCH] Processing {len(batch)} command(s)")
        if self.signals:
            self.signals.batch_started.emit(len(batch))

        valid = []
        has_invalid = False

        for i, angles in enumerate(batch):
            idx = i + 1  # 1-based index for the GUI
            # Only pass the 19 HS mujoco angles to the collision checker — STD servos
            # and play_time sit after index 18 and are not part of the mujoco model.
            mujoco_q = angles[:19]
            if has_self_collision(mujoco_q):
                has_invalid = True
                self.get_logger().warn(f"[BATCH] Item {idx}/{len(batch)} — COLLISION DETECTED: {[round(v,2) for v in mujoco_q]}")
                if self.signals:
                    self.signals.invalid_detected.emit(idx, angles)
            else:
                self.get_logger().info(f"[BATCH] Item {idx}/{len(batch)} — OK")
                valid.append(angles)

            if self.signals:
                self.signals.progress_updated.emit(idx, len(batch))
            
            # Tiny sleep to allow UI to breathe if checking happens too instantly
            time.sleep(0.001)

        # Pause and ask for confirmation if needed
        if has_invalid and len(batch) > 1:
            self.get_logger().warn(f"[BATCH] {len(batch) - len(valid)}/{len(batch)} item(s) had collisions — waiting for GUI confirmation before publishing valid ones")
            if self.signals:
                self.signals.needs_confirmation.emit()
            
            self.proceed_event.clear()
            self.proceed_event.wait() # Thread blocks here until GUI clicks Proceed
            
            if not rclpy.ok(): 
                return

        if valid:
            self.get_logger().info(f"[BATCH] Queuing {len(valid)}/{len(batch)} valid command(s) for publishing")
            self._send_queue.put(valid)

    # ─────────────────────────────────────────────────────────────────
    # Helpers for angle splitting (Task 1)
    # ─────────────────────────────────────────────────────────────────
    @staticmethod
    def _split_mujoco_to_cmd_arrays(angles: list) -> tuple[list, list]:
        """Split a mujoco-safety-ordered angles list (+ trailing play_time) into
        (legs_cmd, upperbody_cmd) ready to publish.

        Input layout (from servo_control_gui.update_servo_position):
            angles[0..18]  : 19 HS servo angles in MUJOCO_SAFETY_ID_ORDER
            angles[19..22] : 4 STD servo angles in STD_SERVO_IDS order (101,102,103,104)
            angles[23]     : play_time (ms)

        Output:
            legs_cmd      : 13 floats  [12 × leg HS angles in LEGS_HS_CMD_IDS order, play_time]
            upperbody_cmd : 12 floats  [7 × upper HS angles in UPPERBODY_HS_CMD_IDS order,
                                        4 × STD servo angles as received, play_time]
        """
        play_time    = float(angles[-1])
        mujoco_angles = angles[:19]       # HS angles
        std_angles    = angles[19:23]     # STD servo angles (101,102,103,104)

        # Build servo_id → angle lookup for HS servos
        id_to_angle = {
            MUJOCO_SAFETY_ID_ORDER[i]: float(mujoco_angles[i])
            for i in range(len(mujoco_angles))
        }

        legs_cmd  = [id_to_angle[sid] for sid in LEGS_HS_CMD_IDS] + [play_time]
        # Upper: 7 HS angles + 4 STD servo angles (passed through as-is) + play_time
        upper_cmd = [id_to_angle[sid] for sid in UPPERBODY_HS_CMD_IDS] + [float(a) for a in std_angles] + [play_time]

        return legs_cmd, upper_cmd

    def _send_worker(self) -> None:
        while rclpy.ok():
            try:
                batch: list[list] = self._send_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if self.signals:
                self.signals.publishing_status.emit("Publishing angles...")

            for angles in batch:
                if not rclpy.ok():
                    break

                try:
                    legs_cmd, upper_cmd = self._split_mujoco_to_cmd_arrays(angles)
                except (KeyError, IndexError) as exc:
                    self.get_logger().error(f"Failed to split mujoco angles: {exc}")
                    continue

                self.get_logger().info(f"[PUBLISH] legs ({len(legs_cmd)} values): {[round(v,2) for v in legs_cmd]}")
                self.get_logger().info(f"[PUBLISH] upper ({len(upper_cmd)} values): {[round(v,2) for v in upper_cmd]}")

                legs_msg = Float32MultiArray()
                legs_msg.data = legs_cmd
                self._legs_cmd_pub.publish(legs_msg)

                upper_msg = Float32MultiArray()
                upper_msg.data = upper_cmd
                self._upperbody_cmd_pub.publish(upper_msg)

                if self._send_delay > 0:
                    time.sleep(self._send_delay)

            if self.signals:
                self.signals.publishing_status.emit("Published all angles")

            self._send_queue.task_done()
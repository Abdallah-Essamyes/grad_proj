#!/usr/bin/env python3
"""
position_republisher.py

Simulates robot motion for testing purposes when the physical robot is absent.
Subscribes to command topics and republishes them as feedback topics,
assuming commanded positions are immediately reached.

Topics:
  Subscribed:  /upperbody_command  (std_msgs/Float32MultiArray)
               /legs_command       (std_msgs/Float32MultiArray)
               /status_command     (std_msgs/Float32MultiArray)
  Published:   /upperbody_feedback (std_msgs/Float32MultiArray)
               /legs_feedback      (std_msgs/Float32MultiArray)

Protocol (status_command):
  CMD_MOVE_ONE = 8  →  data = [8, servo_id, angle, play_time]
    Updates the single servo's slot in the relevant feedback array and republishes.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from pathlib import Path
import sys
try:
    from settings.settings import *
except ModuleNotFoundError:
    parent_dir = str(Path(__file__).resolve().parent.parent)
    # 2. Add it to Python's system path
    sys.path.append(parent_dir)
    from settings.settings import *

_BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Must match STM_microROS.ino motor index arrays exactly
_LEG_SERVO_IDS   = [16, 6, 7, 8, 10, 9, 17, 18, 12, 13, 15, 14]  # 12 entries
_UPPER_SERVO_IDS = [0, 1, 2, 3, 4, 11, 19]                         # 7 entries

_STD_SERVOS_PADDING = [90.0] * 4

CMD_MOVE_ONE = 8


class PositionRepublisher(Node):
    def __init__(self):
        super().__init__('position_republisher')

        self.upperbody_feedback_pub = self.create_publisher(
            UPPERBODY_MSG_TYPE, UPPERBODY_SUB_TOPIC, UPPERBODY_SUB_QOS
        )
        self.legs_feedback_pub = self.create_publisher(
            LEGS_MSG_TYPE, LEGS_SUB_TOPIC, LEGS_SUB_QOS
        )

        # Internal state arrays — updated by all command sources
        self._legs_state  = [0.0] * len(_LEG_SERVO_IDS)
        self._upper_state = [0.0] * len(_UPPER_SERVO_IDS) + _STD_SERVOS_PADDING  # append 4 dummy slots for standard servos (IDs 101-104)

        self.create_subscription(
            UPPERBODY_MSG_TYPE,
            UPPERBODY_PUB_TOPIC,
            self._upperbody_cmd_callback,
            UPPERBODY_PUB_QOS,
        )
        self.create_subscription(
            LEGS_MSG_TYPE,
            LEGS_PUB_TOPIC,
            self._legs_cmd_callback,
            LEGS_PUB_QOS,
        )
        self.create_subscription(
            STATUS_MSG_TYPE,
            STATUS_COMMAND_TOPIC,
            self._status_cmd_callback,
            STATUS_PUB_QOS,
        )

        self.get_logger().info(
            'PositionRepublisher started — forwarding commands to feedback topics.'
        )

        # Publish zero positions immediately on startup
        self._publish_legs()
        self._publish_upper()

        self.get_logger().info('Published zero positions on startup.')

    # ── helpers ────────────────────────────────────────────────────────────────
    def _publish_legs(self):
        msg = Float32MultiArray()
        msg.data = self._legs_state
        self.legs_feedback_pub.publish(msg)
        self.get_logger().debug(f'legs_feedback: {self._legs_state}')

    def _publish_upper(self):
        msg = Float32MultiArray()
        msg.data = list(self._upper_state)  # already includes STD servo slots
        self.upperbody_feedback_pub.publish(msg)
        self.get_logger().debug(f'upperbody_feedback: {self._upper_state}')

    # ── callbacks ──────────────────────────────────────────────────────────────
    def _upperbody_cmd_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        self._upper_state[:len(data)] = data[:len(self._upper_state)]
        self._publish_upper()

    def _legs_cmd_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        self._legs_state[:len(data)] = data[:len(self._legs_state)]
        self._publish_legs()

    def _status_cmd_callback(self, msg: Int16MultiArray):
        if not msg.data:
            return
        cmd = msg.data[0]
        if cmd == CMD_MOVE_ONE and len(msg.data) >= 3:
            servo_id = int(msg.data[1])
            angle    = float(msg.data[2])
            if servo_id in _LEG_SERVO_IDS:
                idx = _LEG_SERVO_IDS.index(servo_id)
                self._legs_state[idx] = angle
                self._publish_legs()
                self.get_logger().debug(
                    f'CMD_MOVE_ONE: legs servo {servo_id} (idx {idx}) → {angle}°'
                )
            elif servo_id in _UPPER_SERVO_IDS:
                idx = _UPPER_SERVO_IDS.index(servo_id)
                self._upper_state[idx] = angle
                self._publish_upper()
                self.get_logger().debug(
                    f'CMD_MOVE_ONE: upper servo {servo_id} (idx {idx}) → {angle}°'
                )


def main(args=None):
    rclpy.init(args=args)
    node = PositionRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

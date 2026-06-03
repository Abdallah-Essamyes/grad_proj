from PyQt6.QtCore import pyqtSignal, QObject
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from settings.settings import *


class ServoControlROSNode(Node, QObject):
    angles_callback_signal = pyqtSignal(str, list)
    torque_feedback_signal = pyqtSignal(list)   # list of 20 torque bools (int 0/1)
    motor_status_signal    = pyqtSignal(list)   # flat list of 40 ints [err0,det0, err1,det1, ...]

    def __init__(self):
        Node.__init__(self, 'servo_gui_ros_node')
        QObject.__init__(self)
        # Explicitly define a Reliable QoS profile
 
        self._dynamic_publishers: dict[str, Publisher] = {}

        self.legs_sub = self.create_subscription(
            LEGS_MSG_TYPE, LEGS_SUB_TOPIC, self.legs_callback, LEGS_SUB_QOS)
        self.legs_pub = self.create_publisher(
            LEGS_MSG_TYPE, LEGS_PUB_TOPIC, LEGS_PUB_QOS)

        self.upperbody_sub = self.create_subscription(
            UPPERBODY_MSG_TYPE, UPPERBODY_SUB_TOPIC, self.upperbody_callback, UPPERBODY_SUB_QOS)
        self.upperbody_pub = self.create_publisher(
            UPPERBODY_MSG_TYPE, UPPERBODY_PUB_TOPIC, UPPERBODY_PUB_QOS)

        self.collision_verification_pub = self.create_publisher(
            COLLISION_VALIDATION_MSG_TYPE, COLLISION_VALIDATION_TOPIC, COLLISION_VALIDATION_PUB_QOS)

        # Unified command publisher (PC -> STM)
        self.status_pub = self.create_publisher(
            STATUS_MSG_TYPE, STATUS_COMMAND_TOPIC, STATUS_PUB_QOS)

        # Unified response subscriber (STM -> PC)
        self.status_sub = self.create_subscription(
            STATUS_MSG_TYPE, STATUS_RESPONSE_TOPIC, self.status_response_callback, STATUS_SUB_QOS)

    # Publishers
    def publish_generic(self, topic_name: str, data_type: type, msg) -> None:
        if topic_name not in self._dynamic_publishers:
            try:
                self._dynamic_publishers[topic_name] = self.create_publisher(data_type, topic_name, 10)
            except Exception as e:
                print(f"Failed to create publisher for {topic_name}: {e}")
                return
        try:
            self._dynamic_publishers[topic_name].publish(msg)
        except Exception as e:
            print(f"Failed to publish to {topic_name}: {e}")
    
    def publish_collision_verification(self, angles: list):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in angles]
        self.collision_verification_pub.publish(msg)

    def publish_legs_angles(self, num: list):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in num]
        self.legs_pub.publish(msg)

    def publish_upperbody_angles(self, num: list):
        msg = Float32MultiArray()
        msg.data = [float(a) for a in num]
        self.upperbody_pub.publish(msg)

    def _send_status_command(self, data: list[int]):
        """Send a status_command array. data[0] is the index byte. Padded to STATUS_ARRAY_SIZE."""
        padded = (data + [0] * STATUS_ARRAY_SIZE)[:STATUS_ARRAY_SIZE]
        msg = Int16MultiArray()
        msg.data = padded
        self.status_pub.publish(msg)

    def request_status(self):
        """Ask STM to read and send back current servo status array (index 0)."""
        self._send_status_command([CMD_REQUEST_STATUS])

    def publish_torque(self, torque_on: bool):
        """Send torque set command (index 2). data[1]: 1=ON, 0=OFF."""
        self._send_status_command([CMD_TORQUE_SET, 1 if torque_on else 0])

    def request_torque_status(self):
        """Ask STM to read and send back current torque status array (index 3)."""
        self._send_status_command([CMD_REQUEST_TORQUE])

    def reset_error(self):
        """Send reset error command (index 6)."""
        self._send_status_command([CMD_RESET_ERROR])

    def reinitialize_servos(self):
        """Send reinitialize command (index 7): STM reboots all servos then runs initialize()."""
        self._send_status_command([CMD_REINITIALIZE])

    def move_one_servo(self, servo_id: int, angle: int, play_time: int = 500):
        """Move a single Herkulex servo (index 8). data[1]=servo_id, data[2]=angle, data[3]=play_time."""
        self._send_status_command([CMD_MOVE_ONE, servo_id, angle, play_time])

    # Subscribers
    def legs_callback(self, msg: Int16MultiArray):
        self.angles_callback_signal.emit(LEGS, list(msg.data))

    def upperbody_callback(self, msg: Int16MultiArray):
        self.angles_callback_signal.emit(UPPERBODY, list(msg.data))

    def status_response_callback(self, msg: Int16MultiArray):
        if not msg.data:
            return
        resp_index = msg.data[0]
        if resp_index == RESP_STATUS_ARRAY:
            # data[1..40] = 20 x [statusError, statusDetail]
            self.motor_status_signal.emit(list(msg.data[1:41]))
        elif resp_index == RESP_TORQUE_ARRAY:
            # data[1..20] = 20 x torque byte
            self.torque_feedback_signal.emit(list(msg.data[1:21]))

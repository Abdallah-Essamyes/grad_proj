from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from pathlib import Path
import sys
try:
    from subClasses.Servo_Pair import Servo_Pair
except ModuleNotFoundError:
    parent_dir = str(Path(__file__).resolve().parent.parent)
    # 2. Add it to Python's system path
    sys.path.append(parent_dir)
    from subClasses.Servo_Pair import Servo_Pair

MUJOCO_SAFETY_ID_ORDER = [16,6,7,8,9,10,17,18,12,13,14,15,19,0,1,2,3,4,11]

# Servo ID ordering for legs and upper-body commands (mirrors command_array in servo_control_gui)
# These must match the STM's motor index arrays exactly.
LEGS_HS_CMD_IDS      = [16, 6, 7, 8, 10, 9, 17, 18, 12, 13, 15, 14]   # 12 Herkulex leg servos
UPPERBODY_HS_CMD_IDS = [0, 1, 2, 3, 4, 11, 19]                          # 7 Herkulex upper-body servos

# Standard (non-Herkulex) servo IDs and their display pin names
STD_SERVO_IDS = [101, 102, 103, 104]
STD_SERVO_DISPLAY: dict[int, str] = {101: "PB13", 102: "PB14", 103: "PB15", 104: "PA8"}
            #pin number in stm itself       29            30          31           8
torque_hotkey = "z"
HS_SERVO_ANGLE_LIMIT = 150 # -150 to 150
STD_SERVO_ANGLE_LIMIT = 160 # 20 to 160

STD_SERVO_MOTION_PAIRS = [Servo_Pair(servo_left_id=104, servo_right_id=103, min_angle=20, max_angle=160, servos_full_range=180,
                                     servo_left_start_angle = 90, servo_right_start_angle = 76)]


BE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

LEGS_SUB_TOPIC      = "/legs_feedback"
LEGS_PUB_TOPIC      = "/legs_command"
LEGS_MSG_TYPE   = Float32MultiArray
LEGS_PUB_QOS = RELIABLE_QOS   # RELIABLE: commands must arrive
LEGS_SUB_QOS = BE_QOS
COLLISION_VALIDATION_TOPIC = "/collision_verification"
COLLISION_VALIDATION_MSG_TYPE = Float32MultiArray
COLLISION_VALIDATION_PUB_QOS = BE_QOS
LEGS                      = "Legs"
UPPERBODY_SUB_TOPIC = "/upperbody_feedback"
UPPERBODY_PUB_TOPIC = "/upperbody_command"
UPPERBODY_MSG_TYPE = Float32MultiArray
UPPERBODY_PUB_QOS = RELIABLE_QOS   # RELIABLE: commands must arrive
UPPERBODY_SUB_QOS = BE_QOS
UPPERBODY                 = "Upperbody"
STATUS_COMMAND_TOPIC        = "/status_command"
STATUS_RESPONSE_TOPIC       = "/status_response"
STATUS_MSG_TYPE           = Float32MultiArray
STATUS_PUB_QOS = RELIABLE_QOS  # RELIABLE: commands must arrive
STATUS_SUB_QOS = BE_QOS

# Index protocol constants (PC -> STM via status_command, data[0])
CMD_REQUEST_STATUS = 0   # request status array
CMD_TORQUE_SET     = 2   # torque change  (data[1]: 1=ON, 0=OFF)
CMD_REQUEST_TORQUE = 3   # request torque status array
CMD_RESET_ERROR    = 6   # reset error
CMD_REINITIALIZE   = 7   # reinitialize all servos (reboot + clearError + ACK + torqueON)
CMD_MOVE_ONE              = 8   # move single Herkulex servo: data[1]=servo_id, data[2]=angle(deg), data[3]=play_time(ms)
COLLISION_DETECTION_FLAG  = 9   # collision flag: data[1]=1 collision active, 0 cleared

# Index protocol constants (STM -> PC via status_response, data[0])
RESP_STATUS_ARRAY  = 1   # status array  (data[1..40] = 20x[statusError, statusDetail])
RESP_TORQUE_ARRAY  = 5   # torque array  (data[1..20] = 20x torque byte)

STATUS_ARRAY_SIZE    = 41  # index byte + up to 40 data bytes
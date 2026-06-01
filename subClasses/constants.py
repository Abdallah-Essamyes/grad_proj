from subClasses.Servo_Pair import Servo_Pair
# Standard (non-Herkulex) servo IDs and their display pin names
STD_SERVO_IDS = {101, 102, 103, 104}
STD_SERVO_DISPLAY: dict[int, str] = {101: "PB13", 102: "PB14", 103: "PB15", 104: "PA8"}
            #pin number in stm itself       29            30          31           8
torque_hotkey = "z"
HS_SERVO_ANGLE_LIMIT = 150 # -150 to 150
STD_SERVO_ANGLE_LIMIT = 160 # 20 to 160

STD_SERVO_MOTION_PAIRS = [Servo_Pair(servo_left_id=104, servo_right_id=103, min_angle=20, max_angle=160, servos_full_range=180,
                                     servo_left_start_angle = 90, servo_right_start_angle = 76)]
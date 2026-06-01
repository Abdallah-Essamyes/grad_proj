#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/u_int8_multi_array.h>
#include <std_msgs/msg/string.h>

// rcutils logging is unreliable on STM32 microROS — use /nubi_debug publisher instead
#define NUM_STD_SERVOS 4
#define NUM_LEGS 12
#define NUM_UPPDERBODY 7
const uint leg_motor_indecies[NUM_LEGS] = {16,6,7,8,10,9,17,18,12,13,15,14};
const uint upper_motor_indecies[NUM_UPPDERBODY] = {0,1,2,3,4,11,19};
// number of motors
const int n=20;
// loops on servos by turn — separate indices for legs and upper body
int leg_feedback_index = 0;
int upper_feedback_index = 0;
#define FEEDBACK_TOTAL 19   // 12 legs + 7 upper (kept for reference)
// Standard (non-Herkulex) servos — pin order matches upperbody_command.data [7..10]
const int std_servo_ids[NUM_STD_SERVOS] = {101, 102, 103, 104};
const int std_servo_pins[NUM_STD_SERVOS] = {PB13, PB14, PB15, PA8};
Servo std_servo[NUM_STD_SERVOS];
// ------------------- Index protocol (status_command / status_response) --------
// PC -> STM (status_command data[0]):
//   0 = request status array
//   2 = torque change  (data[1]: 1=ON, 0=OFF)
//   3 = request torque status array
//   6 = reset error
//   7 = reinitialize servos (reboot all + clearError + ACK + torqueON)
//   8 = move one servo  data[1]=servo_id  data[2]=angle(deg, int16)  data[3]=play_time(ms)
// STM -> PC (status_response data[0]):
//   1 = status array    (data[1..40] = 20 × [statusError, statusDetail])
//   5 = torque array    (data[1..20] = 20 × torque byte)
#define CMD_REQUEST_STATUS   0
#define CMD_TORQUE_SET       2
#define CMD_REQUEST_TORQUE   3
#define CMD_RESET_ERROR      6
#define CMD_REINITIALIZE     7
#define CMD_MOVE_ONE         8
#define RESP_STATUS_ARRAY    1
#define RESP_TORQUE_ARRAY    5
#define STATUS_ARRAY_SIZE     41   // index byte + up to 40 data bytes
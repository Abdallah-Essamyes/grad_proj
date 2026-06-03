#include <micro_ros_arduino.h>
#include "Herkulex.h"
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
#include <Servo.h>
#include <string.h>
#include "constants.h"
// ------------------- micro-ROS objects defined once -------------------
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ------------------- micro-ROS Subscribers object -------------------
rcl_subscription_t leg_command_subscriber;
rcl_subscription_t upperbody_command_subscriber;
rcl_subscription_t status_command_subscriber;
rcl_subscription_t gripper_command_subscriber;
rcl_subscription_t color_cmd_subscriber;

// ------------------- micro-ROS Publishers object -------------------
rcl_publisher_t leg_pos_feedback_publisher;
rcl_publisher_t upperbody_pos_feedback_publisher;
rcl_publisher_t status_response_publisher;
rcl_publisher_t color_feedback_publisher;
rcl_publisher_t debug_publisher;


// -------------------Timer objects -------------------
rcl_timer_t color_timer;
rcl_timer_t collision_warn_timer;

//Subscriber messages
std_msgs__msg__Float32MultiArray legs_command;
std_msgs__msg__Float32MultiArray upperbody_command;
std_msgs__msg__Int16MultiArray   status_command;
std_msgs__msg__Int16MultiArray   gripper_command;
std_msgs__msg__Int16MultiArray   color_command;

//Publisher messages
std_msgs__msg__Float32MultiArray legs_feedback;
std_msgs__msg__Float32MultiArray upperbody_feedback;
std_msgs__msg__Int16MultiArray   status_response;
std_msgs__msg__Int16MultiArray   color_feedback;

// Collision detection flag — set by CMD_COLLISION_DETECTION_FLAG from PC
bool collision_detection_flag = false;

// Debug string message — reused for all log publishes
std_msgs__msg__String debug_msg;
static char debug_char_buf[128];

// Helper: publish a debug string to /nubi_debug
void debug_log(const char* msg) {
  debug_msg.data.data = debug_char_buf;
  debug_msg.data.capacity = sizeof(debug_char_buf);
  strncpy(debug_char_buf, msg, sizeof(debug_char_buf) - 1);
  debug_char_buf[sizeof(debug_char_buf) - 1] = '\0';
  debug_msg.data.size = strlen(debug_char_buf);
  rcl_publish(&debug_publisher, &debug_msg, NULL);
}

// statusError/statusDetail are now local to status_cmd_callback


// macros to check if any function returns anything other than RCL_RET_OK othwerwise stick to error or pass
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// same check but without sticking in error loop
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  // NOTE: do NOT call debug_log here — debug_publisher may not be initialized yet.
  while(1){
    digitalWrite(PC13, !digitalRead(PC13));
    delay(100);
  }
}

// ----- Collision warning timer callback (5 Hz = 200 ms) -----
void collision_warn_callback(rcl_timer_t * timer, int64_t last_call_time){
  (void)last_call_time;
  if(collision_detection_flag){
    debug_log("COLLISION_DETECTION");
  }
}

// --------------------- Subsribers Callback Functions ---------------------
void legs_cmd_callback(const void * msgin){
  // Queue all 12 leg servos then fire simultaneously with actionAll
  int i = 0;
  for(; i<NUM_LEGS;i++){
    Herkulex.moveAllAngle(leg_motor_indecies[i], legs_command.data.data[i], LED_BLUE);
  }
  //always take last index as playtime (float -> int cast)
  Herkulex.actionAll((int)legs_command.data.data[i]);  // 500ms execution time — reduce if hardware allows
}

void upperbody_cmd_callback(const void * msgin){
  // Queue all 7 Herkulex servos then fire simultaneously with actionAll
  int i = 0;
  for(; i<NUM_UPPDERBODY; i++){
    Herkulex.moveAllAngle(upper_motor_indecies[i], upperbody_command.data.data[i], LED_BLUE);
  }
  // i == NUM_UPPDERBODY (7) — next 4 values are standard servo angles (float, constrained to 0..180)
  for(int j = 0; j < NUM_STD_SERVOS; j++){
    int servo_pos = constrain((int)upperbody_command.data.data[i + j], 0, 180);
    std_servo[j].write(servo_pos);
  }
  // last element (data[11]) is playtime for Herkulex actionAll (float -> int cast)
  Herkulex.actionAll((int)upperbody_command.data.data[i + NUM_STD_SERVOS]);
}

void status_cmd_callback(const void * msgin){
  int16_t idx = status_command.data.data[0];

  // ── Print index and indicated action ──
  char log_buf[128];
  const char* action_str = "unknown";
  switch(idx){
    case CMD_REQUEST_STATUS: action_str = "request status array";  break;
    case CMD_TORQUE_SET:     action_str = "torque set";             break;
    case CMD_REQUEST_TORQUE: action_str = "request torque array";  break;
    case CMD_RESET_ERROR:    action_str = "reset error";            break;
    case CMD_REINITIALIZE:   action_str = "reinitialize servos";    break;
    case CMD_MOVE_ONE:       action_str = "move one servo";         break;
    case COLLISION_DETECTION_FLAG: action_str = "collision flag";  break;
  }
  snprintf(log_buf, sizeof(log_buf), "[NUBI] received index %d -> %s", (int)idx, action_str);
  debug_log(log_buf);

  if(idx == CMD_REQUEST_STATUS){
    unsigned long t0 = micros();
    for(int i = 0; i < 20; i++){
      byte statusError = 0, statusDetail = 0;
      byte result = Herkulex.stat(i, statusError, statusDetail);
      if(result != (byte)-1 && result != (byte)-2){
        status_response.data.data[i * 2 + 1] = statusError;
        status_response.data.data[i * 2 + 2] = statusDetail;
      }
    }
    unsigned long elapsed = micros() - t0;
    snprintf(log_buf, sizeof(log_buf), "[NUBI] status read 20 servos: %lu us", elapsed);
    debug_log(log_buf);
    status_response.data.data[0] = RESP_STATUS_ARRAY;
    status_response.data.size = STATUS_ARRAY_SIZE;
    rcl_publish(&status_response_publisher, &status_response, NULL);
  }
  else if(idx == CMD_TORQUE_SET){
    int16_t torque_on = status_command.data.data[1];
    if(torque_on == 1){
      Herkulex.torqueON(BROADCAST_ID);
      debug_log("[NUBI] torqueON applied");
    } else {
      Herkulex.torqueOFF(BROADCAST_ID);
      debug_log("[NUBI] torqueOFF applied");
    }
    // Auto-publish torque state immediately after applying change
    unsigned long t0_tq = micros();
    for(int i = 0; i < 20; i++){
      byte tq = Herkulex.getTorque(i);
      status_response.data.data[i + 1] = (tq == 0x60) ? 1 : 0;
    }
    unsigned long elapsed_tq = micros() - t0_tq;
    snprintf(log_buf, sizeof(log_buf), "[NUBI] torque auto-publish after set: %lu us", elapsed_tq);
    debug_log(log_buf);
    status_response.data.data[0] = RESP_TORQUE_ARRAY;
    status_response.data.size = STATUS_ARRAY_SIZE;
    rcl_publish(&status_response_publisher, &status_response, NULL);
  }
  else if(idx == CMD_REQUEST_TORQUE){
    unsigned long t0 = micros();
    for(int i = 0; i < 20; i++){
      byte tq = Herkulex.getTorque(i);
      status_response.data.data[i + 1] = (tq == 0x60) ? 1 : 0;
    }
    unsigned long elapsed = micros() - t0;
    snprintf(log_buf, sizeof(log_buf), "[NUBI] torque read 20 servos: %lu us", elapsed);
    debug_log(log_buf);
    status_response.data.data[0] = RESP_TORQUE_ARRAY;
    status_response.data.size = STATUS_ARRAY_SIZE;
    rcl_publish(&status_response_publisher, &status_response, NULL);
  }
  else if(idx == CMD_RESET_ERROR){
    Herkulex.clearError(BROADCAST_ID);
    debug_log("[NUBI] clearError applied");
  }
  else if(idx == CMD_MOVE_ONE){
    int16_t servo_id  = status_command.data.data[1];
    int16_t angle     = status_command.data.data[2];
    int16_t play_time = status_command.data.data[3];
    for(int i = 0; i < NUM_STD_SERVOS; i++)
    {
        if(std_servo_ids[i] == servo_id){
            std_servo[i].write(angle);
            char mv_buf[64];
            snprintf(mv_buf, sizeof(mv_buf), "[NUBI] moveOne STD_SERVO id=%d angle=%d t=%d", (int)servo_id, (int)angle, (int)play_time);
            debug_log(mv_buf);
            return; // If it's a standard servo command, we handle it here and return early without calling Herkulex.moveOneAngle
        }
    }   
    Herkulex.moveOneAngle(servo_id, (float)angle, (int)play_time, LED_BLUE);
    char mv_buf[64];
    snprintf(mv_buf, sizeof(mv_buf), "[NUBI] moveOne HS_Servo id=%d angle=%d t=%d", (int)servo_id, (int)angle, (int)play_time);
    debug_log(mv_buf);
  }
  else if(idx == CMD_REINITIALIZE){
    debug_log("[NUBI] reinitialize: rebooting all servos...");
    for(int i = 0; i < n; i++){
      Herkulex.reboot(i);
      delay(50);
    }
    delay(1500);  // wait for all servos to fully boot
    Herkulex.initialize();  // clearError + ACK(1) + torqueON
    delay(200);
    Herkulex.clearError(BROADCAST_ID);
    delay(50);
    Herkulex.torqueON(BROADCAST_ID);
    debug_log("[NUBI] reinitialize complete");
  }
  else if(idx == COLLISION_DETECTION_FLAG){
    collision_detection_flag = (status_command.data.data[1] == 1);
    if(collision_detection_flag){
      debug_log("[NUBI] collision_detection_flag SET");
    } else {
      debug_log("[NUBI] collision_detection_flag CLEARED");
    }
  }
}



// --------------------- Subsribers Setup Functions ---------------------
void leg_cmd_sub_setup(){
  static float memory_buffer[13]; 
  legs_command.data.capacity = 13;
  legs_command.data.data = memory_buffer;
  legs_command.data.size = 0;

  RCCHECK(rclc_subscription_init_default(
    &leg_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "legs_command"));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &leg_command_subscriber, &legs_command, &legs_cmd_callback, ON_NEW_DATA));
}

void upperbody_cmd_sub_setup(){
  // 7 Herkulex + 4 std servo + 1 playtime = 12
  static float memory_buffer1[12]; 
  upperbody_command.data.capacity = 12;
  upperbody_command.data.data = memory_buffer1;
  upperbody_command.data.size = 0;

  RCCHECK(rclc_subscription_init_best_effort(
    &upperbody_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "upperbody_command"));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &upperbody_command_subscriber, &upperbody_command, &upperbody_cmd_callback, ON_NEW_DATA));
}

void status_cmd_sub_setup(){
  static int16_t status_cmd_buffer[STATUS_ARRAY_SIZE];
  status_command.data.capacity = STATUS_ARRAY_SIZE;
  status_command.data.data     = status_cmd_buffer;
  status_command.data.size     = 0;

  RCCHECK(rclc_subscription_init_default(
    &status_command_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "status_command"));

  RCCHECK(rclc_executor_add_subscription(&executor, &status_command_subscriber, &status_command, &status_cmd_callback, ON_NEW_DATA));
}



// Status and torque are now driven by GUI requests (status_command).
// No periodic publish timers needed.


void setup() {
  pinMode(LED_BUILTIN,OUTPUT);

  // NOTE: std_servo.attach() must NOT be called before set_microros_transports().
  // PA8/PB13-15 use TIM1 which conflicts with micro-ROS transport init on STM32.
  // Attach is done after all micro-ROS setup below.

  // Sets up the serial communication (usually USB-Serial or UART) to 
  // talk to the micro-ROS Agent on your PC
  set_microros_transports();
  // put your pinMode definitions here

  delay(2000);

  // get default memory allocator
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  RCCHECK(rclc_node_init_default(&node, "NUBI_STM_NODE", "", &support));

  // Int16MultiArray does not automatically create space to hold the incoming array data.
  // We need space for 12 integers

  // Create a static buffer to hold the data you want to send
  // Initialised to 1004.0f (the "no power" sentinel) so the GUI shows "--" for every
  // servo slot until a real reading arrives. See plans/no-power-sentinel-init.md
  static float feedback_buffer[12];
  for(int i = 0; i < 12; i++) feedback_buffer[i] = 1004.0f;
  // Link the buffer to the message struct
  legs_feedback.data.capacity = 12;
  legs_feedback.data.data = feedback_buffer;
  legs_feedback.data.size = 12; // IMPORTANT: Tell ROS how many items you are sending

  static float feedback_buffer1[7];
  for(int i = 0; i < 7; i++) feedback_buffer1[i] = 1004.0f;
  // Link the buffer to the message struct
  upperbody_feedback.data.capacity = 7;
  upperbody_feedback.data.data = feedback_buffer1;
  upperbody_feedback.data.size = 7; 

  // ── status_response buffer setup ──
  static int16_t status_resp_buffer[STATUS_ARRAY_SIZE];
  memset(status_resp_buffer, 0, sizeof(status_resp_buffer));
  status_response.data.capacity = STATUS_ARRAY_SIZE;
  status_response.data.data     = status_resp_buffer;
  status_response.data.size     = STATUS_ARRAY_SIZE;

  RCCHECK(rclc_publisher_init_best_effort(
    &leg_pos_feedback_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "legs_feedback"));

  RCCHECK(rclc_publisher_init_best_effort(
    &upperbody_pos_feedback_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "upperbody_feedback"));

  rclc_publisher_init_best_effort(
    &status_response_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "status_response");

  rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "nubi_debug");

  RCSOFTCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));  
  leg_cmd_sub_setup();        //1
  upperbody_cmd_sub_setup();  //2
  status_cmd_sub_setup();     //3

  // 5 Hz collision warning timer (200 ms) — logs "COLLISION_DETECTION" to /nubi_debug when flag is set
  RCCHECK(rclc_timer_init_default(&collision_warn_timer, &support, RCL_MS_TO_NS(200), collision_warn_callback));
  RCCHECK(rclc_executor_add_timer(&executor, &collision_warn_timer)); //4

  // Attach standard servos AFTER micro-ROS init to avoid TIM1 conflict
  for(int j = 0; j < NUM_STD_SERVOS; j++){
    std_servo[j].attach(std_servo_pins[j]);
  }

  //Servo initialization
  delay(2000);  //a delay to have time for serial monitor opening
  // NOTE: begin(baud, rx_pin, tx_pin) — PA10=RX (servo TX), PA9=TX (servo RX)
  Herkulex.begin(115200, PA10, PA9); //open serial — rx first, then tx
  for(int i=0; i<n; i++){
    Herkulex.reboot(i); //reboot motors
    delay(50);           // increased: servo needs ~40ms to come back after reboot
  }
  delay(1500);           // wait for ALL servos to fully boot before initialize
  
  Herkulex.initialize(); //initialize motors: clearError + ACK(1) + torqueON
  delay(200);
  // Second clearError+torqueON pass to recover any Break-mode servos
  Herkulex.clearError(BROADCAST_ID);
  delay(50);
  Herkulex.torqueON(BROADCAST_ID);
  delay(100);
}


void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Round-robin: read ONE leg AND ONE upper servo per loop iteration
  // Legs cycle 0..11, upper cycles 0..6 independently
  static unsigned long last_pos_log_ms = 0;
  {
    unsigned long t0 = micros();
    float leg_angle = Herkulex.getAngle(leg_motor_indecies[leg_feedback_index]);
    if (leg_angle < 900.0f) {
      // Valid reading — store float directly
      legs_feedback.data.data[leg_feedback_index] = leg_angle;
    } else if (leg_angle >= 1002.0f) {
      // Timeout sentinel (1004.0f) — servo unpowered/disconnected, propagate so GUI shows "--"
      legs_feedback.data.data[leg_feedback_index] = 1004.0f;
    }
    // 999 (checksum noise) falls through: buffer keeps its last good value silently
    unsigned long el = micros() - t0;
    unsigned long now_ms = millis();
    if (now_ms - last_pos_log_ms >= 1000) {
      char buf[128];
      snprintf(buf, sizeof(buf), "[NUBI] leg[%d] getAngle: %lu us", leg_feedback_index, el);
      debug_log(buf);
      last_pos_log_ms = now_ms;
    }
    leg_feedback_index++;
    if (leg_feedback_index >= 12) leg_feedback_index = 0;
  }

  {
    unsigned long t0 = micros();
    float upper_angle = Herkulex.getAngle(upper_motor_indecies[upper_feedback_index]);
    if (upper_angle < 900.0f) {
      // Valid reading — store float directly
      upperbody_feedback.data.data[upper_feedback_index] = upper_angle;
    } else if (upper_angle >= 1002.0f) {
      // Timeout sentinel (1004.0f) — servo unpowered/disconnected, propagate so GUI shows "--"
      upperbody_feedback.data.data[upper_feedback_index] = 1004.0f;
    }
    // 999 (checksum noise) falls through: buffer keeps its last good value silently
    unsigned long el = micros() - t0;
    unsigned long now_ms = millis();
    if (now_ms - last_pos_log_ms >= 1000) {
      char buf[128];
      snprintf(buf, sizeof(buf), "[NUBI] upper[%d] getAngle: %lu us", upper_feedback_index, el);
      debug_log(buf);
      last_pos_log_ms = now_ms;
    }
    upper_feedback_index++;
    if (upper_feedback_index >= 7) upper_feedback_index = 0;
  }

  // Publish feedback
  RCSOFTCHECK(rcl_publish(&leg_pos_feedback_publisher, &legs_feedback, NULL));
  RCSOFTCHECK(rcl_publish(&upperbody_pos_feedback_publisher, &upperbody_feedback, NULL));
  // status_response is published on demand via status_cmd_callback


  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(4)));
}
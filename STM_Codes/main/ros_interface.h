#pragma once

#include <micro_ros_arduino.h>
#include "constants.h"

// ---------------------------------------------------------------------------
// RosInterface — owns every micro-ROS object: node, executor, subscribers,
// publishers, timers, message buffers, and all callback logic.
//
// Usage in main.ino:
//   RosInterface ros;
//   RosInterface* RosInterface::instance = nullptr;
//
//   void setup() {
//     set_microros_transports();
//     ...hardware init...
//     ros.setup();
//   }
//
//   void loop() {
//     /* write to ros.legs_feedback / ros.upperbody_feedback */
//     ros.publish_feedback();
//     ros.spin();
//   }
// ---------------------------------------------------------------------------
class RosInterface {
public:
    // ── Feedback messages — populated each loop() iteration by main.ino ──
    std_msgs__msg__Float32MultiArray legs_feedback;
    std_msgs__msg__Float32MultiArray upperbody_feedback;

    // Pointer to the single instance; must be set in main.ino before setup().
    static RosInterface* instance;

    // Initialise all ROS objects (call once, after set_microros_transports).
    void setup();

    // Publish a UTF-8 string to /nubi_debug.
    void debug_log(const char* msg);

    // Publish legs_feedback and upperbody_feedback.
    void publish_feedback();

    // Drive the executor (call every loop iteration).
    void spin();

private:
    Servo std_servo[NUM_STD_SERVOS];

    // ── Core micro-ROS infrastructure ──
    rclc_executor_t _executor;
    rclc_support_t  _support;
    rcl_allocator_t _allocator;
    rcl_node_t      _node;

    // ── Subscribers ──
    rcl_subscription_t _leg_sub;
    rcl_subscription_t _upperbody_sub;
    rcl_subscription_t _status_sub;

    // ── Publishers ──
    rcl_publisher_t _leg_pub;
    rcl_publisher_t _upperbody_pub;
    rcl_publisher_t _status_pub;
    rcl_publisher_t _debug_pub;

    // ── Timers ──
    rcl_timer_t _collision_timer;

    // ── Inbound messages ──
    std_msgs__msg__Float32MultiArray _legs_cmd;
    std_msgs__msg__Float32MultiArray _upperbody_cmd;
    std_msgs__msg__Int16MultiArray   _status_cmd;

    // ── Outbound messages ──
    std_msgs__msg__Int16MultiArray   _status_resp;
    std_msgs__msg__String            _debug_msg;

    // ── Collision flag — toggled via CMD_COLLISION_DETECTION_FLAG ──
    bool _collision_flag;

    // ── Raw memory buffers ──
    float   _leg_cmd_buf[13];
    float   _upperbody_cmd_buf[12];
    int16_t _status_cmd_buf[STATUS_ARRAY_SIZE];
    int16_t _status_resp_buf[STATUS_ARRAY_SIZE];
    float   _leg_fb_buf[12];
    float   _upperbody_fb_buf[7];
    char    _debug_char_buf[128];

    // ── Subscriber / timer setup helpers ──
    void _setup_leg_sub();
    void _setup_upperbody_sub();
    void _setup_status_sub();

    // ── Callback implementations ──
    void _on_legs_cmd(const void* msgin);
    void _on_upperbody_cmd(const void* msgin);
    void _on_status_cmd(const void* msgin);
    void _on_collision_warn(rcl_timer_t* timer, int64_t last_call_time);

    // ── Static callback wrappers (plain C pointers required by micro-ROS) ──
    static void _legs_cb(const void* m)                      { instance->_on_legs_cmd(m); }
    static void _upperbody_cb(const void* m)                 { instance->_on_upperbody_cmd(m); }
    static void _status_cb(const void* m)                    { instance->_on_status_cmd(m); }
    static void _collision_cb(rcl_timer_t* t, int64_t l)    { instance->_on_collision_warn(t, l); }
};

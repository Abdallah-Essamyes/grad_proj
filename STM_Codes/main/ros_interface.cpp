#include "ros_interface.h"
#include "Herkulex.h"

// Static instance pointer — assigned in main.ino before ros.setup() is called.
RosInterface* RosInterface::instance = nullptr;

// ---------------------------------------------------------------------------
// File-local helper: return the HerkulexClass that owns the given servo ID.
// IDs in h1[] → Bus 1 (Herkulex / Serial1).
// IDs in h2[] → Bus 2 (Herkulex2 / Serial2).
// Defaults to Herkulex (Bus 1) if the ID is not found in either list.
// ---------------------------------------------------------------------------
static HerkulexClass& busForId(int id) {
    for (int i = 0; i < NUM_H1; i++) if ((int)h1[i] == id) return Herkulex;
    for (int i = 0; i < NUM_H2; i++) if ((int)h2[i] == id) return Herkulex2;
    return Herkulex;  // fallback
}

// ===========================================================================
// Public API
// ===========================================================================

void RosInterface::setup() {
    instance = this;
    _collision_flag = false;

    _allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&_support, 0, NULL, &_allocator));
    RCCHECK(rclc_node_init_default(&_node, "NUBI_STM_NODE", "", &_support));

    // ── Feedback buffer initialisation (1004.0 = "no power" sentinel) ──
    for (int i = 0; i < 12; i++) _leg_fb_buf[i]       = 1004.0f;
    for (int i = 0; i < 7;  i++) _upperbody_fb_buf[i] = 1004.0f;

    legs_feedback.data.capacity = 12;
    legs_feedback.data.data     = _leg_fb_buf;
    legs_feedback.data.size     = 12;

    upperbody_feedback.data.capacity = 7;
    upperbody_feedback.data.data     = _upperbody_fb_buf;
    upperbody_feedback.data.size     = 7;

    // ── Status response buffer ──
    memset(_status_resp_buf, 0, sizeof(_status_resp_buf));
    _status_resp.data.capacity = STATUS_ARRAY_SIZE;
    _status_resp.data.data     = _status_resp_buf;
    _status_resp.data.size     = STATUS_ARRAY_SIZE;

    // ── Publishers ──
    RCCHECK(rclc_publisher_init_best_effort(
        &_leg_pub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "legs_feedback"));

    RCCHECK(rclc_publisher_init_best_effort(
        &_upperbody_pub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "upperbody_feedback"));

    rclc_publisher_init_best_effort(
        &_status_pub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "status_response");

    rclc_publisher_init_default(
        &_debug_pub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "nubi_debug");

    // ── Executor — 4 handles: 3 subscribers + 1 timer ──
    RCSOFTCHECK(rclc_executor_init(&_executor, &_support.context, 4, &_allocator));

    _setup_leg_sub();       // handle 1
    _setup_upperbody_sub(); // handle 2
    _setup_status_sub();    // handle 3

    // 5 Hz collision warning timer (200 ms)
    RCCHECK(rclc_timer_init_default(
        &_collision_timer, &_support,
        RCL_MS_TO_NS(200), _collision_cb));
    RCCHECK(rclc_executor_add_timer(&_executor, &_collision_timer)); // handle 4
}

void RosInterface::debug_log(const char* msg) {
    _debug_msg.data.data     = _debug_char_buf;
    _debug_msg.data.capacity = sizeof(_debug_char_buf);
    strncpy(_debug_char_buf, msg, sizeof(_debug_char_buf) - 1);
    _debug_char_buf[sizeof(_debug_char_buf) - 1] = '\0';
    _debug_msg.data.size = strlen(_debug_char_buf);
    rcl_publish(&_debug_pub, &_debug_msg, NULL);
}

void RosInterface::publish_feedback() {
    RCSOFTCHECK(rcl_publish(&_leg_pub,      &legs_feedback,      NULL));
    RCSOFTCHECK(rcl_publish(&_upperbody_pub, &upperbody_feedback, NULL));
}

void RosInterface::spin() {
    RCSOFTCHECK(rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(4)));
}

// ===========================================================================
// Private — subscriber setup helpers
// ===========================================================================

void RosInterface::_setup_leg_sub() {
    _legs_cmd.data.capacity = 13;
    _legs_cmd.data.data     = _leg_cmd_buf;
    _legs_cmd.data.size     = 0;

    RCCHECK(rclc_subscription_init_default(
        &_leg_sub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "legs_command"));

    RCCHECK(rclc_executor_add_subscription(
        &_executor, &_leg_sub, &_legs_cmd, _legs_cb, ON_NEW_DATA));
}

void RosInterface::_setup_upperbody_sub() {
    // 7 Herkulex + 4 std servos + 1 playtime = 12
    _upperbody_cmd.data.capacity = 12;
    _upperbody_cmd.data.data     = _upperbody_cmd_buf;
    _upperbody_cmd.data.size     = 0;

    RCCHECK(rclc_subscription_init_best_effort(
        &_upperbody_sub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "upperbody_command"));

    RCCHECK(rclc_executor_add_subscription(
        &_executor, &_upperbody_sub, &_upperbody_cmd, _upperbody_cb, ON_NEW_DATA));
}

void RosInterface::_setup_status_sub() {
    _status_cmd.data.capacity = STATUS_ARRAY_SIZE;
    _status_cmd.data.data     = _status_cmd_buf;
    _status_cmd.data.size     = 0;

    RCCHECK(rclc_subscription_init_best_effort(
        &_status_sub, &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "status_command"));

    RCCHECK(rclc_executor_add_subscription(
        &_executor, &_status_sub, &_status_cmd, _status_cb, ON_NEW_DATA));
}

// ===========================================================================
// Private — callback implementations
// ===========================================================================

void RosInterface::_on_legs_cmd(const void* /*msgin*/) {
    // h1 motors occupy indices 0..NUM_H1-1 in the command array
    for (int i = 0; i < NUM_H1; i++) {
        Herkulex.moveAllAngle(h1[i], _legs_cmd.data.data[i], LED_BLUE);
    }
    // remaining leg motors (indices NUM_H1..NUM_LEGS-1) are on Bus 2
    for (int i = NUM_H1; i < NUM_LEGS; i++) {
        Herkulex2.moveAllAngle(leg_motor_indecies[i], _legs_cmd.data.data[i], LED_BLUE);
    }
    int ptime = (int)_legs_cmd.data.data[NUM_LEGS];  // last element is playtime
    Herkulex.actionAll(ptime);
    Herkulex2.actionAll(ptime);
}

void RosInterface::_on_upperbody_cmd(const void* /*msgin*/) {
    // All Herkulex upper-body motors are on Bus 2 (h2[0..NUM_UPPDERBODY-1])
    for (int i = 0; i < NUM_UPPDERBODY; i++) {
        Herkulex2.moveAllAngle(h2[i], _upperbody_cmd.data.data[i], LED_BLUE);
    }
    for (int j = 0; j < NUM_STD_SERVOS; j++) {
        int servo_pos = constrain((int)_upperbody_cmd.data.data[NUM_UPPDERBODY + j], 0, 180);
        std_servo[j].write(servo_pos);
    }
    int ptime = (int)_upperbody_cmd.data.data[NUM_UPPDERBODY + NUM_STD_SERVOS];
    Herkulex2.actionAll(ptime);
}

void RosInterface::_on_status_cmd(const void* /*msgin*/) {
    int16_t idx = _status_cmd.data.data[0];

    char log_buf[128];
    const char* action_str = "unknown";
    switch (idx) {
        case CMD_REQUEST_STATUS:        action_str = "request status array";  break;
        case CMD_TORQUE_SET:            action_str = "torque set";             break;
        case CMD_REQUEST_TORQUE:        action_str = "request torque array";   break;
        case CMD_RESET_ERROR:           action_str = "reset error";            break;
        case CMD_REINITIALIZE:          action_str = "reinitialize servos";    break;
        case CMD_MOVE_ONE:              action_str = "move one servo";         break;
        case COLLISION_DETECTION_FLAG:  action_str = "collision flag";         break;
    }
    snprintf(log_buf, sizeof(log_buf), "[NUBI] received index %d -> %s", (int)idx, action_str);
    debug_log(log_buf);

    if (idx == CMD_REQUEST_STATUS) {
        unsigned long t0 = micros();
        for (int i = 0; i < NUM_H1; i++) {
            byte statusError = 0, statusDetail = 0;
            byte result = Herkulex.stat(h1[i], statusError, statusDetail);
            if (result != (byte)-1 && result != (byte)-2) {
                _status_resp.data.data[h1[i] * 2 + 1] = statusError;
                _status_resp.data.data[h1[i] * 2 + 2] = statusDetail;
            }
        }
        for (int i = 0; i < NUM_H2; i++) {
            byte statusError = 0, statusDetail = 0;
            byte result = Herkulex2.stat(h2[i], statusError, statusDetail);
            if (result != (byte)-1 && result != (byte)-2) {
                _status_resp.data.data[h2[i] * 2 + 1] = statusError;
                _status_resp.data.data[h2[i] * 2 + 2] = statusDetail;
            }
        }
        snprintf(log_buf, sizeof(log_buf),
                 "[NUBI] status read 20 servos: %lu us", micros() - t0);
        debug_log(log_buf);
        _status_resp.data.data[0] = RESP_STATUS_ARRAY;
        _status_resp.data.size    = STATUS_ARRAY_SIZE;
        rcl_publish(&_status_pub, &_status_resp, NULL);
    }
    else if (idx == CMD_TORQUE_SET) {
        int16_t torque_on = _status_cmd.data.data[1];
        if (torque_on == 1) {
            Herkulex.torqueON(BROADCAST_ID);
            Herkulex2.torqueON(BROADCAST_ID);
            debug_log("[NUBI] torqueON applied");
        } else {
            Herkulex.torqueOFF(BROADCAST_ID);
            Herkulex2.torqueOFF(BROADCAST_ID);
            debug_log("[NUBI] torqueOFF applied");
        }
        unsigned long t0 = micros();
        for (int i = 0; i < NUM_H1; i++) {
            byte tq = Herkulex.getTorque(h1[i]);
            _status_resp.data.data[h1[i] + 1] = (tq == 0x60) ? 1 : 0;
        }
        for (int i = 0; i < NUM_H2; i++) {
            byte tq = Herkulex2.getTorque(h2[i]);
            _status_resp.data.data[h2[i] + 1] = (tq == 0x60) ? 1 : 0;
        }
        snprintf(log_buf, sizeof(log_buf),
                 "[NUBI] torque auto-publish after set: %lu us", micros() - t0);
        debug_log(log_buf);
        _status_resp.data.data[0] = RESP_TORQUE_ARRAY;
        _status_resp.data.size    = STATUS_ARRAY_SIZE;
        rcl_publish(&_status_pub, &_status_resp, NULL);
    }
    else if (idx == CMD_REQUEST_TORQUE) {
        unsigned long t0 = micros();
        for (int i = 0; i < NUM_H1; i++) {
            byte tq = Herkulex.getTorque(h1[i]);
            _status_resp.data.data[h1[i] + 1] = (tq == 0x60) ? 1 : 0;
        }
        for (int i = 0; i < NUM_H2; i++) {
            byte tq = Herkulex2.getTorque(h2[i]);
            _status_resp.data.data[h2[i] + 1] = (tq == 0x60) ? 1 : 0;
        }
        snprintf(log_buf, sizeof(log_buf),
                 "[NUBI] torque read 20 servos: %lu us", micros() - t0);
        debug_log(log_buf);
        _status_resp.data.data[0] = RESP_TORQUE_ARRAY;
        _status_resp.data.size    = STATUS_ARRAY_SIZE;
        rcl_publish(&_status_pub, &_status_resp, NULL);
    }
    else if (idx == CMD_RESET_ERROR) {
        Herkulex.clearError(BROADCAST_ID);
        Herkulex2.clearError(BROADCAST_ID);
        debug_log("[NUBI] clearError applied");
    }
    else if (idx == CMD_MOVE_ONE) {
        int16_t servo_id  = _status_cmd.data.data[1];
        int16_t angle     = _status_cmd.data.data[2];
        int16_t play_time = _status_cmd.data.data[3];
        for (int i = 0; i < NUM_STD_SERVOS; i++) {
            if (std_servo_ids[i] == servo_id) {
                std_servo[i].write(angle);
                char mv_buf[64];
                snprintf(mv_buf, sizeof(mv_buf),
                         "[NUBI] moveOne STD_SERVO id=%d angle=%d t=%d",
                         (int)servo_id, (int)angle, (int)play_time);
                debug_log(mv_buf);
                return;
            }
        }
        busForId(servo_id).moveOneAngle(servo_id, (float)angle, (int)play_time, LED_BLUE);
        char mv_buf[64];
        snprintf(mv_buf, sizeof(mv_buf),
                 "[NUBI] moveOne HS_Servo id=%d angle=%d t=%d",
                 (int)servo_id, (int)angle, (int)play_time);
        debug_log(mv_buf);
    }
    else if (idx == CMD_REINITIALIZE) {
        debug_log("[NUBI] reinitialize: rebooting all servos...");
        for (int i = 0; i < NUM_H1; i++) { Herkulex.reboot(h1[i]);  delay(50); }
        for (int i = 0; i < NUM_H2; i++) { Herkulex2.reboot(h2[i]); delay(50); }
        delay(1500);
        Herkulex.initialize();
        Herkulex2.initialize();
        delay(200);
        Herkulex.clearError(BROADCAST_ID);
        Herkulex2.clearError(BROADCAST_ID);
        delay(50);
        Herkulex.torqueON(BROADCAST_ID);
        Herkulex2.torqueON(BROADCAST_ID);
        debug_log("[NUBI] reinitialize complete");
    }
    else if (idx == COLLISION_DETECTION_FLAG) {
        _collision_flag = (_status_cmd.data.data[1] == 1);
        debug_log(_collision_flag
                  ? "[NUBI] collision_detection_flag SET"
                  : "[NUBI] collision_detection_flag CLEARED");
    }
}

void RosInterface::_on_collision_warn(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
    if (_collision_flag) {
        debug_log("COLLISION_DETECTION");
    }
}

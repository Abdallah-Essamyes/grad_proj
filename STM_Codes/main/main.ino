#include <Arduino.h>
#include <micro_ros_arduino.h>
#include "Herkulex.h"
#include <Servo.h>
#include <string.h>
#include "constants.h"
// Round-robin feedback indices — one per bus, independent of body grouping
int h1_feedback_index = 0;
int h2_feedback_index = 0;
#include "ros_interface.h"

// Single ROS interface instance — owns all micro-ROS objects and callbacks.
RosInterface ros;


// ===========================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    // NOTE: std_servo.attach() must NOT be called before set_microros_transports().
    // PA8/PB13-15 use TIM1 which conflicts with micro-ROS transport init on STM32.
    // Attach is done after all micro-ROS setup below.
    set_microros_transports();

    delay(2000);

    // Initialise all ROS subscribers, publishers, timers, and the executor.
    ros.setup();

    // ── Herkulex hardware initialisation ──
    delay(2000);  // wait for serial monitor before first UART traffic
    Herkulex.begin(BAUD_RATE, PA10, PA9);                  // Bus 1 — Serial1 (TX=PA9,  RX=PA10)
    Herkulex2.begin(BAUD_RATE, Serial2, PA3, PA2);         // Bus 2 — Serial2 (TX=PA2,  RX=PA3)
    for (int i = 0; i < NUM_H1; i++) { Herkulex.reboot(h1[i]);  delay(50); }
    for (int i = 0; i < NUM_H2; i++) { Herkulex2.reboot(h2[i]); delay(50); }
    delay(1500);  // wait for all servos to fully boot

    Herkulex.initialize();   // Bus 1: clearError + ACK(1) + torqueON
    Herkulex2.initialize();  // Bus 2: clearError + ACK(1) + torqueON
    delay(200);
    // Second pass — recovers any servos that booted in Break-mode.
    Herkulex.clearError(BROADCAST_ID);
    Herkulex2.clearError(BROADCAST_ID);
    delay(50);
    Herkulex.torqueON(BROADCAST_ID);
    Herkulex2.torqueON(BROADCAST_ID);
    delay(100);
}

// ===========================================================================
void loop() {
    // Round-robin: read ONE servo from Bus 1 and ONE from Bus 2 per iteration.
    // Both requests fire on separate buses simultaneously (get2positions), so
    // we only block for one response window instead of two.
    static unsigned long last_pos_log_ms = 0;
    {
        unsigned long t0 = micros();
        float h1_angle = 1004.0f;
        float h2_angle = 1004.0f;

        get2positions(
            h1[h1_feedback_index],
            h2[h2_feedback_index],
            h1_angle,
            h2_angle);

        // Store Bus 1 result -> legs_feedback (h1 = leg servos)
        if      (h1_angle <  900.0f)  ros.legs_feedback.data.data[h1_feedback_index] = h1_angle;
        else if (h1_angle >= 1002.0f) ros.legs_feedback.data.data[h1_feedback_index] = 1004.0f;

        // Store Bus 2 result -> upperbody_feedback (h2 = upper-body servos)
        if      (h2_angle <  900.0f)  ros.upperbody_feedback.data.data[h2_feedback_index] = h2_angle;
        else if (h2_angle >= 1002.0f) ros.upperbody_feedback.data.data[h2_feedback_index] = 1004.0f;

        unsigned long now_ms = millis();
        if (now_ms - last_pos_log_ms >= 1000) {
            char buf[128];
            snprintf(buf, sizeof(buf), "[NUBI] get2pos h1[%d] h2[%d]: %lu us",
                     h1[h1_feedback_index], h2[h2_feedback_index], micros() - t0);
            ros.debug_log(buf);
            last_pos_log_ms = now_ms;
        }

        h1_feedback_index++;
        if (h1_feedback_index >= NUM_H1) h1_feedback_index = 0;
        h2_feedback_index++;
        if (h2_feedback_index >= NUM_H2) h2_feedback_index = 0;
    }

    ros.publish_feedback();
    ros.spin();
}

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

    // ── Herkulex hardware initialisation ──
    // MUST happen before ros.setup() so the micro-ROS session is not left
    // idle for ~5 s while servos reboot — that exceeds the agent keepalive
    // timeout and causes the STM32 to endlessly reconnect.
    // Herkulex uses Serial1/Serial2 (UART), not TIM1, so it is safe to init
    // here before std_servo.attach() (which is deferred into ros.setup()).
    delay(2000);  // allow servos to power up before first UART traffic
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

    // Initialise all ROS subscribers, publishers, timers, and the executor.
    // Session is established here; loop() starts spinning immediately after.
    ros.setup();
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

        // Store each servo's angle into the correct feedback array at the correct index.
        // h1/h2 are WIRING groups (not command-order groups) — they contain a mix of
        // leg and upper-body servos. We look up each servo ID in leg_motor_indecies and
        // upper_motor_indecies to find the right array and slot.
        // This also fixes a buffer overflow: h2 has 9 servos but upperbody_feedback
        // only has NUM_UPPDERBODY=7 slots, so blindly using h2_feedback_index caused
        // writes at indices 7 and 8 into a 7-element array.
        {
            const int   bus_ids[2]  = { h1[h1_feedback_index], h2[h2_feedback_index] };
            const float bus_ang[2]  = { h1_angle,              h2_angle              };
            for (int b = 0; b < 2; b++) {
                float ang = bus_ang[b];
                // Only update the buffer on a valid reading (< 900°).
                // On timeout (1004) or checksum error (999), keep the last
                // known-good value so the GUI never flashes "--" mid-operation.
                if (ang >= 900.0f) continue;
                int  sid    = bus_ids[b];
                bool stored = false;
                for (int k = 0; k < NUM_LEGS && !stored; k++) {
                    if ((int)leg_motor_indecies[k] == sid) {
                        ros.legs_feedback.data.data[k] = ang; stored = true;
                    }
                }
                for (int k = 0; k < NUM_UPPDERBODY && !stored; k++) {
                    if ((int)upper_motor_indecies[k] == sid) {
                        ros.upperbody_feedback.data.data[k] = ang; stored = true;
                    }
                }
            }
        }

        unsigned long now_ms = millis();
        if (now_ms - last_pos_log_ms >= 1000) {
            char buf[128];
            // Cast to int: 1004=timeout, 999=checksum error, else real angle in degrees
            snprintf(buf, sizeof(buf), "[NUBI] h1[%d]=%d  h2[%d]=%d  (%lu us)",
                     h1[h1_feedback_index], (int)h1_angle,
                     h2[h2_feedback_index], (int)h2_angle,
                     micros() - t0);
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

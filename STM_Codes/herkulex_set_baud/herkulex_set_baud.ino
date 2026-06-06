/*
 * herkulex_set_baud.ino
 *
 * Sets the baud rate of all Herkulex servos (IDs 0–18) to 666666 bps.
 *
 * The baud rate is stored in EEP (non-volatile) register address 4
 * (official Herkulex manual table):
 *   0x02 = 666,666  <-- target
 *   0x03 = 500,000
 *   0x04 = 400,000
 *   0x07 = 250,000
 *   0x09 = 200,000
 *   0x10 = 115,200  (factory default)
 *   0x22 =  57,600
 *
 * After writing, each servo is rebooted so the new baud rate takes effect.
 *
 * Hardware:  Arduino Mega (uses Serial1 @ current baud rate to reach the servos)
 * Wire:      Herkulex data line to Mega TX1/RX1 pins via half-duplex adapter.
 *
 * IMPORTANT: Run this sketch at the servos' CURRENT baud rate.
 *            Change CURRENT_BAUD below if your servos are not at 115200.
 */

#include "Herkulex.h"

// ------------------------------------------------------------------
// Configuration
// ------------------------------------------------------------------
#define CURRENT_BAUD  115200   // baud rate the servos are currently using
#define TARGET_BAUD_REG  0x02  // EEP register value for 666,666 bps (official manual)
#define EEP_BAUD_ADDRESS    4  // EEP register address for baud rate (Herkulex manual p.25)

#define FIRST_ID   0
#define LAST_ID   18
// ------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("Herkulex baud-rate setter starting...");
  Serial.print("Connecting to servos at current baud: ");
  Serial.println(CURRENT_BAUD);

  // Connect to the servo bus using Serial1 (Mega hardware UART)
  Herkulex.beginSerial1(CURRENT_BAUD);
  delay(200);

  Herkulex.initialize();  // broadcast torque-off + clear errors
  delay(200);

  Serial.println("Scanning IDs 0 to 18 and updating baud rate...");
  Serial.println();

  for (int id = FIRST_ID; id <= LAST_ID; id++) {
    Serial.print("  Servo ID ");
    Serial.print(id);
    Serial.print(" -> writing EEP[4] = 0x07 (666666 bps) ... ");

    // Write new baud-rate value to EEP (non-volatile memory)
    Herkulex.writeRegistryEEP(id, EEP_BAUD_ADDRESS, TARGET_BAUD_REG);
    delay(100);  // allow EEP write to complete

    // Reboot so the servo loads the new value
    Herkulex.reboot(id);
    delay(500);  // wait for the servo to reboot

    Serial.println("done.");
  }

  Serial.println();
  Serial.println("All done!");
  Serial.println("Reconnect to the servos at 666666 bps to verify.");
}

void loop() {
  // Nothing to do after setup
}

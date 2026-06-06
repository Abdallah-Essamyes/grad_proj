/*
 * herkulex_set_baud.ino
 *
 * Sets the baud rate of all Herkulex servos (h1 + h2 buses) to 666666 bps.
 *
 * Servo IDs mirror constants.h in main/:
 *   h1 (Bus 1 / Serial1): {0,1,2,6,7,8,9,10,16,19}
 *   h2 (Bus 2 / Serial2): {3,4,11,12,13,14,15,18,17}
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
 * Hardware:  Arduino Mega (Serial1 = Bus 1, Serial2 = Bus 2)
 * Wire:      h1 data line to TX1/RX1; h2 data line to TX2/RX2 via half-duplex adapters.
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

// Servo ID lists — must match constants.h in main/
// h1: IDs wired to Bus 1 / Serial1 (TX=PA9,  RX=PA10)
// h2: IDs wired to Bus 2 / Serial2 (TX=PA2,  RX=PA3)
const int h1[] = {0, 1, 2, 6, 7, 8, 9, 10, 16, 19};
const int h2[] = {3, 4, 11, 12, 13, 14, 15, 18, 17};
#define NUM_H1 (int)(sizeof(h1) / sizeof(h1[0]))
#define NUM_H2 (int)(sizeof(h2) / sizeof(h2[0]))
// ------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("Herkulex baud-rate setter starting...");
  Serial.print("Connecting to servos at current baud: ");
  Serial.println(CURRENT_BAUD);
  Serial.print("h1 servo count: "); Serial.println(NUM_H1);
  Serial.print("h2 servo count: "); Serial.println(NUM_H2);
  Serial.println();
  Herkulex.begin(CURRENT_BAUD, PA10, PA9);                   // Bus 1 — Serial1 (TX=PA9,  RX=PA10)
  Herkulex2.begin(CURRENT_BAUD, Serial2, PA3, PA2);          // Bus 2 — Serial2 (TX=PA2,  RX=PA3)
  delay(200);

  Herkulex.initialize();   // Bus 1: broadcast torque-off + clear errors
  Herkulex2.initialize();  // Bus 2: broadcast torque-off + clear errors
  delay(200);

  // ── Bus 1: h1 servos ──────────────────────────────────────────
  Serial.println("Bus 1 (Serial1) — updating h1 servos...");
  for (int i = 0; i < NUM_H1; i++) {
    int id = h1[i];
    Serial.print("  Servo ID ");
    Serial.print(id);
    Serial.print(" -> writing EEP[4] = 0x02 (666666 bps) ... ");

    Herkulex.writeRegistryEEP(id, EEP_BAUD_ADDRESS, TARGET_BAUD_REG);
    delay(100);

    Herkulex.reboot(id);
    delay(500);

    Serial.println("done.");
  }

  // ── Bus 2: h2 servos ──────────────────────────────────────────
  Serial.println();
  Serial.println("Bus 2 (Serial2) — updating h2 servos...");
  for (int i = 0; i < NUM_H2; i++) {
    int id = h2[i];
    Serial.print("  Servo ID ");
    Serial.print(id);
    Serial.print(" -> writing EEP[4] = 0x02 (666666 bps) ... ");

    Herkulex2.writeRegistryEEP(id, EEP_BAUD_ADDRESS, TARGET_BAUD_REG);
    delay(100);

    Herkulex2.reboot(id);
    delay(500);

    Serial.println("done.");
  }

  Serial.println();
  Serial.println("All done!");
  Serial.println("Reconnect to the servos at 666666 bps to verify.");
}

void loop() {
  // Nothing to do after setup
}

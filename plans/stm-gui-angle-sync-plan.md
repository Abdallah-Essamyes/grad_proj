## Plan: STM ↔ GUI Angle Sync Fixes

After reading the firmware ([STM_Codes/main/main.ino](STM_Codes/main/main.ino), [ros_interface.cpp](STM_Codes/main/ros_interface.cpp), [ros_interface.h](STM_Codes/main/ros_interface.h), [constants.h](STM_Codes/main/constants.h)) and every Python consumer ([subClasses/ros_node.py](subClasses/ros_node.py), [subClasses/collisions.py](subClasses/collisions.py), [servo_control_gui.py](servo_control_gui.py), [Widgets/json_widget.py](Widgets/json_widget.py), [helper_scripts/position_republisher.py](helper_scripts/position_republisher.py), [settings/settings.py](settings/settings.py)), the index→servo-ID contract is in fact **already consistent** on both sides — `legs_feedback[k]` corresponds to `leg_motor_indecies[k]`/`LEGS_HS_CMD_IDS[k]` and `upperbody_feedback[k]` corresponds to `upper_motor_indecies[k]`/`UPPERBODY_HS_CMD_IDS[k]`. The "sync issues" the user is observing are caused by five concrete defects: (1) a wrong message type that makes the collision flag never reach the STM, (2) firmware that holds last-good angles forever when a servo is unplugged mid-run so the GUI silently shows stale data, (3) a round-robin loop that refreshes each individual servo only ~10 Hz at best so a moving robot's GUI visibly trails, (4) a desynced simulator + several stale Int16 references (legacy scripts and the protocol doc all claim Int16 while the live firmware uses Float32), and (5) missing defensive size checks in the firmware command decoders.

This plan fixes them in five small, independently-committable phases, strictly TDD where the code is Python and runtime-verifiable where the code is firmware.

**Phases**

1. **Phase 1: Fix broken collision-flag publish (Int16 → Float32)**
    - **Objective:** Make `CommandVerifier._check_live_collision` actually deliver `COLLISION_DETECTION_FLAG` to the STM. Today it constructs an `Int16MultiArray` and publishes it on a `Float32MultiArray` publisher → rclpy raises `TypeError("Expected Float32MultiArray, got Int16MultiArray")` and the firmware's `_collision_flag` is never set, so the 5 Hz collision warning timer is dead.
    - **Files/Functions to Modify/Create:**
        - [subClasses/collisions.py](subClasses/collisions.py) — `_check_live_collision`: build a `Float32MultiArray` instead, with `data = [float(COLLISION_DETECTION_FLAG), 1.0 if colliding else 0.0]`. Drop the unused `Int16MultiArray` import (or keep it for `_status_cmd_callback`'s legacy hint, but stop using it for this publish).
        - Add a regression test under `tests/test_collisions_publish.py` that constructs a stub publisher, calls a refactored helper that returns the constructed `Float32MultiArray`, and asserts the type, length, and `data[0]==COLLISION_DETECTION_FLAG`.
    - **Tests to Write:**
        - `test_collision_flag_message_is_float32multiarray`
        - `test_collision_flag_payload_layout` (data[0]=flag id, data[1]=1.0 / 0.0, len ≥ 2)
        - `test_collision_flag_publishes_on_status_command_topic` (mock the publisher and assert .publish() called once with the expected msg)
    - **Steps:**
        1. Write the three failing tests against the existing `collisions.py`. Run; observe failures (current behaviour produces the wrong type).
        2. Refactor `_check_live_collision` so the final `flag_msg` construction is in a tiny helper `_build_collision_flag_msg(colliding: bool) -> Float32MultiArray` (this lets the tests run without bringing up rclpy).
        3. Replace `Int16MultiArray()` with `Float32MultiArray()` and cast values to `float`.
        4. Re-run tests; all green. Lint/format.

2. **Phase 2: Replace "last-good forever" with stale-detection sentinel in firmware**
    - **Objective:** When a servo stops responding mid-operation (cable yanked, power dipped), the firmware currently keeps publishing its last known angle as if everything were fine ([main.ino](STM_Codes/main/main.ino#L80-L89) — `if (ang >= 900.0f) continue;`). The GUI cannot tell the difference between "still alive at 42°" and "disconnected, last seen at 42°". Fix: track per-servo consecutive-failure count; after N (default 5) misses in a row, write the 1004 sentinel into the feedback slot so the GUI displays `θ: --` and `set_angle(None)` clears the torque indicator.
    - **Files/Functions to Modify/Create:**
        - [STM_Codes/main/main.ino](STM_Codes/main/main.ino) — extend the round-robin block: maintain `static uint8_t miss_count[NUM_H1+NUM_H2]` keyed by bus-index pair; on success reset to 0, on `ang ≥ 900` increment, on `miss_count >= STALE_THRESHOLD` find the slot via the existing `leg_motor_indecies` / `upper_motor_indecies` lookup and write `1004.0f` there.
        - [STM_Codes/main/constants.h](STM_Codes/main/constants.h) — add `#define FEEDBACK_STALE_THRESHOLD 5` and `#define FEEDBACK_NO_POWER_SENTINEL 1004.0f`.
    - **Tests to Write:** (firmware can't be unit-tested locally without a board; this phase relies on physical verification)
        - Hardware checklist (manual, post-flash):
            - With all servos powered, every widget shows a numeric angle within 2 s of starting both nodes.
            - Unplug one Herkulex servo's data line. Within `STALE_THRESHOLD * loop_period` (~0.5–1 s) the corresponding GUI widget switches to `θ: --`.
            - Re-plug the servo; widget recovers a numeric angle within one full bus cycle (~1 s).
    - **Steps:**
        1. Add `FEEDBACK_STALE_THRESHOLD` / `FEEDBACK_NO_POWER_SENTINEL` to `constants.h`.
        2. In `loop()`, after the existing `if (ang >= 900.0f) continue;` branch, replace the bare `continue` with logic that increments a `miss_count[bus_index_global]` and, on threshold reach, writes the sentinel into the correct slot via the same `leg_motor_indecies` / `upper_motor_indecies` linear search.
        3. On a successful read (ang < 900), reset that servo's `miss_count` to 0.
        4. Compile and flash. Run the manual checklist above. Iterate if needed.

3. **Phase 3: Reduce GUI feedback latency by reading multiple servos per loop**
    - **Objective:** Today [main.ino](STM_Codes/main/main.ino#L65-L101) reads exactly one servo per bus per `loop()` iteration. With ~3–5 ms per Herkulex `getPosition` round-trip plus `ros.spin()`, each individual servo refreshes only ~5–10 Hz. Bumping it to e.g. 3 servos per bus per loop (sequential on the same bus, parallel across buses via `get2positions`) refreshes each servo at ~15–30 Hz — visibly smoother in the GUI without overwhelming the executor.
    - **Files/Functions to Modify/Create:**
        - [STM_Codes/main/main.ino](STM_Codes/main/main.ino) — wrap the existing round-robin block in a `for (int p = 0; p < FEEDBACK_READS_PER_LOOP; p++) { … }` loop, advancing the bus indices each iteration. Keep `get2positions()` so both buses still fire concurrently.
        - [STM_Codes/main/constants.h](STM_Codes/main/constants.h) — `#define FEEDBACK_READS_PER_LOOP 3` (tune empirically; 3 ≈ 9–15 ms which is well under the 200 ms collision-warn timer).
        - [STM_Codes/main/ros_interface.cpp](STM_Codes/main/ros_interface.cpp) — `spin()`: bump `RCL_MS_TO_NS(4)` to `RCL_MS_TO_NS(2)` so each loop spends less time spinning when the queue is empty (optional refinement; revert if subscriber callbacks lose throughput).
    - **Tests to Write:** (firmware — manual)
        - Hardware checklist:
            - With a single servo continuously commanded `±sin(t)` from a Python publisher, the GUI's angle label updates visibly smoother than before (subjective, but should be ~2–3× faster).
            - Latency measurement: drive a servo to a new angle via `move_one_servo`; the GUI label should reach the commanded value within `play_time + (NUM_H1 / FEEDBACK_READS_PER_LOOP)*loop_period` ≈ `play_time + 0.5 s`.
            - The 1 Hz `[NUBI] get2pos h1[..] h2[..]: us` debug log line continues to fire (means the throttle in the inner block still uses absolute `millis()`).
    - **Steps:**
        1. Add `FEEDBACK_READS_PER_LOOP` to `constants.h`.
        2. Wrap the existing round-robin block in a `for (int p=0; p<FEEDBACK_READS_PER_LOOP; p++) { … }`. Keep the 1 Hz debug log inside but only print on the first iteration of each `loop()` (`if (p == 0) …`) to avoid spam.
        3. Optionally tune `spin()` window down to 2 ms.
        4. Compile, flash, run manual checklist; if any servo errors out (CRC storms because the bus is over-utilised) reduce `FEEDBACK_READS_PER_LOOP` to 2.

4. **Phase 4: Harden contracts (simulator, defensive sizes, stale Int16 references, doc)**
    - **Objective:** Bring everything in the repo onto the same wire contract so future regressions are caught at the boundary, not in the GUI.
    - **Files/Functions to Modify/Create:**
        - [helper_scripts/position_republisher.py](helper_scripts/position_republisher.py) — drop `_STD_SERVOS_PADDING`. `self._upper_state = [0.0] * len(_UPPER_SERVO_IDS)` (length 7), making the simulator's `upperbody_feedback` exactly match the firmware's. Update `_upperbody_cmd_callback` so it copies only the first 7 elements (it already does via slice; keep it). Fix `_status_cmd_callback`'s type hint from `Int16MultiArray` to `Float32MultiArray`.
        - [STM_Codes/main/ros_interface.cpp](STM_Codes/main/ros_interface.cpp) — `_on_legs_cmd`, `_on_upperbody_cmd`, `_on_status_cmd`: prefix each with a guard like `if (_legs_cmd.data.size < NUM_LEGS + 1) { debug_log("[NUBI] legs_cmd too short"); return; }` to prevent OOB reads if a future publisher omits the playtime.
        - [STM_Codes/movementPlayback.py](STM_Codes/movementPlayback.py), [STM_Codes/status_sub.py](STM_Codes/status_sub.py), [STM_Codes/torque_sub.py](STM_Codes/torque_sub.py) — change `Int16MultiArray` to `Float32MultiArray` (or delete the legacy scripts after confirming they're unreferenced; `STM_Codes/movementPlayback.py` is duplicated in `old_codes/movementPlayback.py`).
        - [documents/nubi_protocol.md](documents/nubi_protocol.md) — update header `Message type: std_msgs/Int16MultiArray` to `Message type: std_msgs/Float32MultiArray`. Update the layout description to clarify integer values are still expected (cast to int16 inside the firmware) but transported as float32.
    - **Tests to Write:**
        - `tests/test_position_republisher_contract.py`:
            - `test_upperbody_feedback_published_with_seven_elements`
            - `test_legs_feedback_published_with_twelve_elements`
            - `test_move_one_command_routes_to_correct_slot` (parametrized for one leg id and one upper id)
        - Firmware defensive checks: hardware-test by publishing a deliberately short message (`ros2 topic pub --once /legs_command std_msgs/Float32MultiArray "data: [0,0,0]"`) and confirming the chip logs `[NUBI] legs_cmd too short` instead of resetting.
    - **Steps:**
        1. Write the failing simulator tests (use the rclpy `MultiThreadedExecutor` + a memo subscriber pattern, or factor out a pure helper that returns the message and unit-test that). Run; observe the 11-element fail.
        2. Patch `position_republisher.py`. Re-run; tests green.
        3. Patch the three legacy `STM_Codes/*_sub.py` and the protocol doc. Lint.
        4. Add the firmware size guards. Compile.
        5. Flash; run the firmware-side defensive check.

5. **Phase 5 (Optional): Single source of truth for servo ID arrays**
    - **Objective:** Today `leg_motor_indecies` lives in [STM_Codes/main/constants.h](STM_Codes/main/constants.h#L25) and is duplicated in three Python files ([settings/settings.py#L17](settings/settings.py#L17), [servo_control_gui.py#L27](servo_control_gui.py#L27), [helper_scripts/position_republisher.py#L43](helper_scripts/position_republisher.py#L43)). Same for `upper_motor_indecies`, `std_servo_ids`, `STATUS_ARRAY_SIZE`. Any hand-edit in one place that misses another *creates* exactly the kind of sync bug the user is asking us to prevent.
    - **Files/Functions to Modify/Create:**
        - New `documents/servo_ids.yaml` (or `.json`) — single canonical definition: `{ legs: [...], upper: [...], std: [...], h1_wiring: [...], h2_wiring: [...], num_h1, num_h2, status_array_size }`.
        - New `helper_scripts/generate_servo_ids.py` — reads the YAML and emits both [STM_Codes/main/constants.h](STM_Codes/main/constants.h) (or a partial header `servo_ids.generated.h` included from `constants.h`) and a Python module `settings/servo_ids.py` re-exported by `settings/settings.py`.
        - Update [servo_control_gui.py](servo_control_gui.py), [settings/settings.py](settings/settings.py), [helper_scripts/position_republisher.py](helper_scripts/position_republisher.py) to import from the generated module.
        - Update `README.md` with a one-line `python helper_scripts/generate_servo_ids.py` step before reflashing the STM.
    - **Tests to Write:**
        - `tests/test_servo_ids_consistency.py`: parse the generated header with a regex and assert it matches `LEGS_HS_CMD_IDS` etc. exactly.
        - `tests/test_no_hardcoded_id_arrays.py`: grep for `[16, 6, 7, 8, 10, 9, 17, 18, 12, 13, 15, 14]` literal in the codebase and assert it appears only in the YAML and generated files.
    - **Steps:**
        1. Write both consistency tests against current code. Run; expect both to fail (the literal appears in 4 files).
        2. Author the YAML and the generator.
        3. Run the generator; verify `git diff` shows only formatting/comment changes in the generated outputs.
        4. Replace duplicates with imports. Re-run tests. Green.

**Open Questions**

1. Phase 3's `FEEDBACK_READS_PER_LOOP`: start at 3 (preferred, ~3× faster), 2 (safer), or make it a runtime parameter via `status_command`?
2. Phase 4: delete legacy `STM_Codes/movementPlayback.py` and `old_codes/movementPlayback.py` outright, or just patch the message type?
3. Phase 5 is optional refactor — execute now, defer to a follow-up plan, or skip?
4. Phase 2's stale threshold: 5 misses (~0.5–1 s lag before "--") or 3 (~0.3–0.5 s, more flicker risk)?

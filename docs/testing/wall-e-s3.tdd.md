# Wall-E S3 motion-control TDD evidence

## Source plan

No plan file was supplied. User journeys were derived from the request:

1. As a user, I want voice tools for forward, backward, turning, and stop, so the robot can move
   safely.
2. As a user, I want voice tools for eyes, arms, neck, and head yaw, so the robot can express
   itself.
3. As an operator, I want bounded durations, speed limits, angle limits, and an emergency stop,
   so a bad command cannot leave the motors running indefinitely.

## RED/GREEN evidence

The first test run was intentionally RED because the new motion-model and board files did not yet
exist: `python3 -m unittest scripts.tests.test_wall_e_motion -v` failed with missing
`wall_e_motion_model.h`, `wall_e_motion_model.cc`, and `wall_e_s3.cc`.

After implementation, the same command passed. The final repository test suite also passed:

```text
python3 -m unittest discover -s scripts/tests -q
Ran 113 tests in 31.983s
OK
```

## Guarantees

| # | Guarantee | Test | Result |
|---:|---|---|---|
| 1 | Servo angles are clamped and converted to bounded PCA9685 ticks; reversal is supported | `scripts/tests/test_wall_e_motion.py` host C++ test | PASS |
| 2 | Motor mixing handles stop, forward, backward, and both turns; speed is capped at 100% | same host C++ test | PASS |
| 3 | Drive duration is clamped to 200–5000 ms | same host C++ test | PASS |
| 4 | Unknown drive and pose names are rejected | same host C++ test | PASS |
| 5 | The board registers drive, stop, pose, and status MCP tools | board-source assertions | PASS |
| 6 | The board is discoverable as `wall-e-s3` and is restricted to ESP32-S3 | board-menu/build tests | PASS |

## Known gaps

- ESP-IDF was not installed or exported in the execution environment, so an ESP32-S3 firmware
  compile and hardware smoke test could not be run. The proving command failed before configure with
  `RuntimeError: ESP-IDF version was not detected`.
- Physical pin assignments for L298P and the PCA9685 address are documented assumptions in
  `main/boards/wall-e-s3/config.h`; verify them against the actual wiring before applying motor
  power.
- Mechanical servo limits and left/right direction must be calibrated with horns detached and
  external servo power disconnected during the first signal test.

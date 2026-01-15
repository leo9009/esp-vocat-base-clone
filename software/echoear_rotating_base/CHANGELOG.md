# ChangeLog

## V1.0.4 - 2026-01-15

* Optimize the BMM150 sampling rate.
* Optimize the single-click event detection logic of the magnetic sliding switch.

## V1.0.3 - 2025-12-24

* Add fish detached event and pairing cancelled event detection in magnetic slide switch component
* Add initial position detection for magnetic slide switch on power-on

# ChangeLog

## V1.0.2 - 2025-12-19

* Fix the sound source recognition angle control logic
* Update the thresholds when using QMC6309 for the sliding switch

## V1.0.1 - 2025-12-18

**Update the magnetic sliding switch component:**
* Add feed-fish event and detection event for two bases approaching each other face-to-face (for pairing)
* Optimize single-click event detection logic
* Optimize the automatic calibration program
* Update README documentation

## V1.0.0 - 2025-12-02

**First Official Release**

### Features:

* Stepper motor control with acceleration/deceleration and predefined actions (shake head, look around, beat swing, cat nuzzle)
* Magnetic slide switch detection with auto-calibration (support Hall sensor, BMM150, QMC6309)
* UART communication protocol for angle control and event notification
* Base auto-calibration with limit switch on startup
* NVS storage for persistent calibration data

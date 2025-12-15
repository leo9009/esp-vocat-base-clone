# ChangeLog

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

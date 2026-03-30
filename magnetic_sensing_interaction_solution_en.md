# Magnetic Sensing Interaction Solution Introduction

[中文](./magnetic_sensing_interaction_solution_zh.md) | [English](./magnetic_sensing_interaction_solution_en.md)

## 1. Overview

The ESP-VoCat base uses a magnetic slide switch to provide rich interaction control. Different slider positions change the local magnetic field, and the firmware continuously samples sensor data to detect slider actions. Detected events are reported to ESP-VoCat via UART.

## 2. Event Types by Demo Project

### 2.1 Bell Slider Event Detection Demo
Project: [`software/esp_vocat_rotating_base_bell_event_detection`](./software/esp_vocat_rotating_base_bell_event_detection/)

Supported events:

- `SLIDE_DOWN`: Slider is moved from the upper position to the lower position.
- `SLIDE_UP`: Slider is moved from the lower position to the upper position.
- `REMOVE_FROM_UP`: Slider is removed from the upper position.
- `REMOVE_FROM_DOWN`: Slider is removed from the lower position.
- `PLACE_FROM_UP`: Slider is placed into the upper position.
- `PLACE_FROM_DOWN`: Slider is placed into the lower position.
- `SINGLE_CLICK`: A single-click action is detected when the slider stays at the lower position.

### 2.2 iPhone Detection Demo
Project: [`software/esp_vocat_rotating_base_iphone_detection`](./software/esp_vocat_rotating_base_iphone_detection/)

Supported events:

- `IPHONE_LEAN_FRONT`: iPhone leans against the front side of the base.
- `IPHONE_LEAN_FRONT_DETACHED`: iPhone leaves the front side of the base.
- `IPHONE_UNDER_BASE`: iPhone is placed under the base and enters a valid detected state.
- `IPHONE_UNDER_BASE_DETACHED`: iPhone is removed from under the base.

### 2.3 Magnetic Accessory Detection Demo
Project: [`software/esp_vocat_rotating_base_magnetic_accessory_detection`](./software/esp_vocat_rotating_base_magnetic_accessory_detection/)

Supported events:

- `FISH_ATTACHED`: Fish accessory is detected as attached.
- `FISH_DETACHED`: Fish accessory is removed from the base.
- `PAIRING`: Pairing condition is detected and pairing mode is entered.
- `PAIRING_CANCELLED`: Pairing condition disappears and pairing mode is exited.
- `ICE_CREAM_ATTACHED`: Ice-cream accessory is detected as attached.
- `ICE_CREAM_DETACHED`: Ice-cream accessory is removed from the base.
- `DONUT_ATTACHED`: Donut accessory is detected as attached.
- `DONUT_DETACHED`: Donut accessory is removed from the base.

> Note: Event sets are defined by the `magnetic_slide_switch` profile selected by each demo. The source code enum definition is the final reference.

## 3. Sensor Support

One of the following sensors can be used:

- BMM150 3-axis magnetometer (default)
- QMC6309 3-axis magnetometer
- Linear Hall sensor

## 4. Notes

1. Different demo projects expose different event sets. Switch project to switch behavior.
2. Run full auto-calibration before first use to capture baseline values for up/down/removed states.
3. Calibration data is persisted in NVS and survives power cycles.
4. Recalibration can be triggered by UART command or by long-pressing the base Boot button.

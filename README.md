# ESP-VoCat Smart Rotating Base

English | [简体中文](README_CN.md)

## Project Overview

The ESP-VoCat rotating base is a smart rotating platform designed specifically for the [**ESP-VoCat development kit**](https://docs.espressif.com/projects/esp-dev-kits/zh_CN/latest/esp32s3/esp-vocat/user_guide_v1.2.html), tailored for toys, smart speakers, smart hubs, and other products requiring large language model voice interaction capabilities. The device uses the ESP32-C61-WROOM-1 module, supports USB Type-C power supply, and can power the ESP-VoCat main unit through a magnetic interface.

This project implements **high-precision stepper motor control, magnetic slide switch event detection, CSI sensing capabilities**, and stable **UART communication** as core functions. It can automatically adjust direction based on ESP-VoCat's **Sound Source Localization** results, enabling intelligent rotation toward sound sources. By combining motion control with sound source perception, it provides a more **natural and immersive** human-computer interaction experience.

## Key Features

- **Precise Angle Control** - High-precision rotational positioning based on stepper motor
- **Magnetic Slide Switch** - Multiple sensor support with automatic calibration
- **Rich Preset Actions** - Natural movements like shake head, look around, beat rhythm
- **Auto Homing** - Automatic zero position finding and calibration on startup
- **Bidirectional Communication** - Real-time data exchange between controller and base
- **Data Persistence** - Calibration data saved, survives power loss

## Project Structure

```
esp-vocat-base/
├── hardware/                    # Hardware design files
│   ├── schematics/              # Schematics
│   └── pcb/                     # PCB design files
├── software/                    # Firmware code
│   ├── common_components/       # Shared components for all examples
│   │   ├── stepper_motor/               # Stepper motor control
│   │   ├── magnetic_slide_switch/       # Magnetic switch (profile-based)
│   │   ├── control_serial/              # Serial communication
│   │   └── BMM150_SensorAPI/            # Sensor driver
│   ├── esp_vocat_rotating_base/                         # Base firmware
│   ├── esp_vocat_rotating_base_bell_event_detection/    # Bell slider event demo
│   ├── esp_vocat_rotating_base_iphone_detection/        # iPhone detection demo
│   └── esp_vocat_rotating_base_magnetic_accessory_detection/ # Accessory detection demo
├── README.md                    # English documentation
└── README_CN.md                 # Chinese documentation
```

## Example Projects

To avoid frequent branch switching, `software/` includes 4 independently buildable example projects:

- `esp_vocat_rotating_base`  
  Base firmware with standard capabilities: stepper actions, core magnetic slider events, UART communication, calibration, and status sync.

- `esp_vocat_rotating_base_bell_event_detection`  
  Bell slider event detection demo, focused on bell-related magnetic slider event logic.

- `esp_vocat_rotating_base_iphone_detection`  
  iPhone detection demo, focused on phone approach/leave and under-base related detection events.

- `esp_vocat_rotating_base_magnetic_accessory_detection`  
  Magnetic accessory detection demo, focused on accessory magnetic event recognition (e.g. fish/ice-cream/donut style accessories).

All projects share common components under `software/common_components/`.  
The `magnetic_slide_switch` component selects different profiles per project to keep event definitions and detection behavior compatible.

## **Feature Showcase**

[**Video Showcase**](https://www.bilibili.com/video/BV19gSEBwEoj/?spm_id_from=333.1365.list.card_archive.click&vd_source=e731e982043e3ccbb2c03395e0a66c39)

The ESP-VoCat base uses Espressif's **ESP32-C61-WROOM-1(N8R2)** module and communicates with the ESP-VoCat main unit via a custom UART protocol. In terms of mechanical structure, the base is equipped with a 24BYJ48 stepper motor, enabling sound source localization and various predefined rotation actions to enhance interaction experience.
Additionally, the ESP-VoCat base features a magnetic slide switch that unlocks more interactive experiences.

### **Sound Source Localization**
ESP-VoCat supports sound source localization, capable of real-time detection of sound direction and position in the environment. The system collects sound signals through a microphone array and analyzes sound intensity, phase, and other information to determine the azimuth of the sound source. Combined with an ultra-quiet stepper motor-based rotating base, it enables intelligent interaction facing the sound source.

![Sound Source Localization](https://image.lceda.cn/oshwhub/pullImage/d77175cd4dd0484cb67eff6eaa344257.gif)

### **Multiple Preset Actions**
The rotating base supports various predefined actions:

- **Shake Head Action**
The shake head action makes the base gently swing the companion left and right, simulating a cat's curious or subtle gesture.

![Shake Head](https://image.lceda.cn/oshwhub/pullImage/d6aad38613fe448f9a7dd80cf92317be.gif)

- **Curious Look Around**
The curious look around action makes the base move the companion's head left and right, with small random offsets, simulating a **cat naturally observing its environment**.

![Look Around](https://image.lceda.cn/oshwhub/pullImage/f5252b32170644bb90c492240ee94bf2.gif)

- **Beat Rhythm**
The beat rhythm action enables ESP-VoCat to swing its head left and right according to **drum beats** in external music, creating an **interactive effect** synchronized with music.

![Beat Following](https://image.lceda.cn/oshwhub/pullImage/f96c3a6d28774487ba1fabf218944f31.gif)

- **Gentle Nuzzle**
The gentle nuzzle simulates a cat's tender nuzzling action: the base slowly turns left and returns to center, repeating several times. The motion is smooth and natural, with each pause enhancing the **tactile feel** and **gentleness**.

![Nuzzle](https://image.lceda.cn/oshwhub/pullImage/ca70534cecc342eea3f5e9ee2383a7a7.gif)

  **Notes:**
  All actions are **parametrically controllable**, facilitating integration with large models like **Doubao** and **Xiaozhi**.
  To better execute actions, the base's motor control features:
  - **Precise Angle Control:** Supports rotation at any angle with precision up to 0.1 degrees, serving sound source localization
  - **Smooth Acceleration/Deceleration:** Uses acceleration algorithms for natural and smooth rotation
  - **Auto Homing Calibration:** 
  Since the ESP-VoCat rotating base is driven by a stepper motor, it has no angle feedback. Therefore, the base performs zero position calibration at startup:
   1. After power-on, the motor automatically rotates left to find the limit switch
   2. After touching the limit switch, the motor rotates right 95° to the center position
   3. After calibration is complete, the motor powers off

### **Magnetic Slide Switch Interaction Control**
The ESP-VoCat base implements multiple interaction controls through a **magnetic** slide switch. Different slider positions change the magnetic field strength around the magnetometer. The base identifies slider actions by monitoring these magnetic field changes in real-time. When position changes are detected, the base reports corresponding events to ESP-VoCat via serial port, enabling rich and intuitive interactive experiences.

Supports 9 types of magnetic slide switch event detection:
   - Slider moved from up to down (`SLIDE_DOWN`): Slider is moved from the upper position to the lower position.
   - Slider moved from down to up (`SLIDE_UP`): Slider is moved from the lower position to the upper position.
   - Slider removed from up position (`REMOVE_FROM_UP`): Slider is removed from the upper position.
   - Slider removed from down position (`REMOVE_FROM_DOWN`): Slider is removed from the lower position.
   - Slider placed at up position (`PLACE_FROM_UP`): Slider is placed into the upper position.
   - Slider placed at down position (`PLACE_FROM_DOWN`): Slider is placed into the lower position.
   - Single click action (`SINGLE_CLICK`): A single-click action is detected when the slider stays at the lower position.
   - Fish feeding event (`FISH_ATTACHED`): Fish accessory is detected as attached.
   - Pairing mode (`PAIRING`): Two ESP-VoCat devices are face-to-face and pairing condition is detected.

![Magnetic Switch](https://image.lceda.cn/oshwhub/pullImage/351af8952a324622841299e806176f17.gif)

Magnetic slide switch control supports **multiple sensors**:
 - BMM150 3-axis magnetometer
 - QMC6309 3-axis magnetometer
 - Linear Hall sensor

  **Notes:**
  1. **Choose one** of the above sensors; **BMM150** is selected by default. When using a linear Hall sensor, **only** slider up/down sliding (SLIDE_UP / SLIDE_DOWN) event detection is supported!
  2. Before first use, **fully automatic calibration** of the magnetic slide switch is required to adapt to the magnetic environment, recording magnetic field strength baseline values for three states: **up / down / removed**:
     - System automatically detects three stable positions (keep still for about 500ms each position)
     - No manual prompts required, system automatically judges position differences
     - Calibration data persistently stored via NVS Flash, survives power loss
     - During use, recalibration can be triggered by sending a serial command from ESP-VoCat or long-pressing the base's Boot button

Based on magnetometer detection results combined with intelligent motion determination algorithms, the base can recognize nine types of actions and events.

### **CSI Sensing Capability**
The companion can perceive environmental changes through Wi-Fi CSI (Channel State Information), enabling action triggering or environmental interaction.
- **Movement Detection** 
By obtaining CSI information from routers through the companion, human movement throughout the entire space can be detected, enabling human movement detection and data statistics.

- **Near-field Perception** 
Through packet exchange between the companion and base, Wi-Fi transmission paths can be controlled to achieve small-range, high-precision detection. For example, when a human finger is detected approaching, the base can automatically adjust posture or execute preset actions, enabling smarter interactive experiences.

![CSI Sensing](https://image.lceda.cn/oshwhub/pullImage/54420a264bb946bb982edc07daac1b5c.gif)

### **Serial Communication**
The ESP-VoCat rotating base communicates with ESP-VoCat via **UART protocol**.

- **Command Types**:
    - Angle control command (CMD_BASE_ANGLE_CONTROL)
    - Preset action control command (CMD_BASE_ACTION_CONTROL)
    - Preset action completion notification (CMD_ACTION_COMPLETE)
    - Magnetic switch event report (CMD_MAGNETIC_SWITCH_EVENT)
    - Magnetic data calibration progress sync (MAG_SWITCH_CALIB_*)

Note: Currently only the above basic command types are available; CSI-related commands will be added later.
    
-  **Data Frame Format**:
    -   `AA 55 [LEN_H] [LEN_L] [CMD] [DATA...] [CHECKSUM]` 
-  **Verification Mechanism**:
    - **Frame header** verification
    - **Checksum** verification

[**Detailed command data frame example**](software/esp_vocat_rotating_base/README.md)

## **Hardware Requirements**

### **Main Control Chip**
- **ESP32-C61-WROOM-1** or **ESP32-C61-WROOM-1U** module

ESP32-C61-WROOM-1 is used by default

- [**Hardware Open Source**](https://oshwhub.com/esp-college/esp-echoear-base)

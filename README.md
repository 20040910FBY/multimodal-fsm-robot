# Multimodal Perception Autonomous Mobile Robot 

An advanced, multi-modal autonomous mobile robot integrating high-frequency line following, dual-channel PID light tracking, and heuristic depth-matrix obstacle avoidance governed by a strict hierarchical Finite State Machine (FSM).

---

## 📌 Project Overview 

* **Role **: Project Leader 
* **Timeline** : 2024.3 - 2024.9
* **Description **:
  * **EN**: Designed and implemented an end-to-end autonomous navigation and control framework for robotic platforms operating in unstructured or adversarial environments. The system resolves common edge-case failures—such as structural track derailment, light interference oscillation, and local kinematic deadlocks—by fusing asynchronous ADC multi-channel data, deploying dual-channel decoupled controllers, and orchestrating a fault-tolerant heuristic FSM decision engine.

---

## 🚀 Key Technical Highlights 

### 1. Hierarchical Finite State Machine (FSM) 

* **EN**: Engineered a strict behavioral priority arbitration network (`Obstacle Avoidance [P1]` > `Light Tracking [P2]` > `Line Following [P3]`). The system defaults to high-frequency line exploration, seamlessly hot-swaps to ambient light tracking upon target validation, and immediately preempts with reactive escape subroutines when spatial proximity hazards are intercepted.

### 2. High-Frequency Asynchronous ADC & Error Latching 

* **EN**: Upgraded legacy digital GPIO tracking to a 16-bit high-resolution ADS1115 ADC pipeline running over a `400kHz` I2C bus at `860 SPS`. Formulated binary bitmask characterization matrix for precise black-line edge extraction. Integrated a **historical error-latching mechanism**; upon complete track derailment (`B000`), the robot references its last validated state trajectory to execute dead-reckoning attitude correction and path re-entry.

### 3. Dual-Channel Decoupled PID Light Tracking 

* **EN**: Configured a differential perception matrix utilizing symmetric photoresistors with an integrated hardware offset component. Deployed dual independent tracking PID loops running at a fixed `20ms` interval. Incorporated dynamic speed mapping and boundary clipping constraints to bound total control effort, successfully dampening overshoot oscillations under highly non-linear environmental lighting.

### 4. Depth Matrix Cloud Scanning & Heuristic Escape

* **EN**: Developed a dynamic panoramic scan routine pairing a high-speed micro-servo with a time-of-flight ultrasonic transceiver. Utilized frame-difference damping filters to suppress high-frequency acoustic noise. Implemented a heuristic decision algorithm that evaluates multi-angle depth vectors, executing reverse-gear divergence and differential spin maneuvers toward the maximum topological clearance vector to prevent local kinematic deadlocks.

### 5. Adaptive Tolerance Drive Calibration

* **EN**: Compensated for deterministic DC motor manufacturing tolerances and asymmetric friction profiles by introducing real-time software scaling coefficients (`EN1_RATE`, `EN2_RATE`), ensuring flawless linear trajectory execution without closed-loop encoders.

---

## 🛠️ Hardware Topology & Pin Map

| Component | Sub-module | Arduino Pin | Technical Profile |
| :--- | :--- | :--- | :--- |
| **MCU** | Arduino Uno / Nano | - | ATmega328P, 16MHz |
| **Motor Drive** | Left Motor PWM (EN1) | `Pin 6` | Software Calibrated (`EN1_RATE = 1.0`) |
| | Left Direction (IN1, IN2) | `Pin 9`, `Pin 8` | Logic Level Direction H-Bridge |
| | Right Motor PWM (EN2) | `Pin 5` | Software Calibrated (`EN2_RATE = 0.95`)|
| | Right Direction (IN3, IN4) | `Pin 3`, `Pin 2` | Logic Level Direction H-Bridge |
| **Perception Array**| Panoramic Micro-Servo | `Pin 12` | PWM Duty-Cycle Angle Control (45° - 135°) |
| | Ultrasonic Trig / Echo | `A1` / `A0` | Distance Decoupling (Trigger-Pulse Width) |
| | High-Resolution ADC | `A4 (SDA)`, `A5 (SCL)`| ADS1115 I2C Bus @ 400kHz |
| | Light Matrix (L / R) | `A2` / `A3` | Symmetric Photoresistor Differential Input |

---

## 📂 System Architecture

```
                     [ PERCEPTION LAYER /  ]
     +-------------------+--------------------+-------------------+
     |                   |                    |                   |
 [Ultrasonic SR04]  [Photoresistor Matrix] [IR Reflective Array]
     |                   |                    |                   |
 (Distance Data)     (Light Gradients)    (High-res Analog V)
     |                   |                    |                   |
     v                   v                    v                   |
     |                   |             [ADS1115 ADC @ 400kHz]     |
     |                   |                    |                   |
     +-------------------+--------------------+-------------------+
                         |
                         v
                     [ DECISION LAYER ]
              +---------------------------------------+
              |   Heuristic State Machine Engine      |
              +---------------------------------------+
              |  IF Dist < 40cm  -> [OBSTACLE AVOID]  |  (Priority 1)
              |  ELSE IF ΔLight  -> [LIGHT TRACKING]  |  (Priority 2)
              |  ELSE            -> [LINE FOLLOWING]  |  (Priority 3)
              +---------------------------------------+
                         |
        +----------------+----------------+
        |                |                |
        v                v                v
 [Obstacle Avoid]  [Light Tracking]  [Line Following]
 * Braking         * Dual PID Loop   * Bitmask Decoding
 * PTZ Depth Scan  * Speed Mapping   * Error Latching
 * Heuristic Spin  * Clip Constraints* Auto-Correction
        |                |                |
        +----------------+----------------+
                         |
                         v
                     [ EXECUTION LAYER  ]
              +---------------------------------------+
              | Motor Control (With Tolerance Coeffs) |
              +---------------------------------------+
```

---

## 🚀 Getting Started 

### 📦 Prerequisites 

Ensure you have the following libraries installed in your Arduino IDE:

1. `Wire.h` (Built-in I2C core library )
2. `Servo.h` (Built-in PWM servo library)
3. `PID_v1` (By Brett Beauregard)
4. `ADS1115_WE` (By Wolfgang Ewald )

### 🔧 Configuration Notes 

* **I2C Bus Acceleration**: The source code forces the I2C clock to `400kHz` (`Wire.setClock(400000);`) to reduce the ADC conversion blocking window. Ensure short signal leads to prevent I2C bus noise.
* **Calibration Coefficients**: If your chassis exhibits a different drift profile, tune `#define EN2_RATE` in the preamble to perfectly realign the tracking center.

---

## 📜 License 

This repository is released under the [MIT License](LICENSE). Feel free to adapt for educational or research purposes.

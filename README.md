[README_advanced_motor.md](https://github.com/user-attachments/files/26646765/README_advanced_motor.md)
# 🤖 Advanced Motor Control — Dual MCU I2C System

![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![I2C](https://img.shields.io/badge/Protocol-I2C-a855f7?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-a855f7?style=for-the-badge)

> **A dual-microcontroller embedded system for precise DC motor speed and position control, featuring I2C inter-MCU communication, keypad input, and LCD feedback.**

---

## 📖 About

This project takes motor control to the next level by splitting responsibilities across **two Arduino microcontrollers** communicating over the **I2C protocol**. The master MCU reads user input from a keypad, then relays commands to the slave MCU which executes the motor control logic.

This architecture mirrors real-world embedded systems design — where tasks are distributed across multiple processors for modularity, efficiency, and scalability. If you're looking to understand MCU-to-MCU communication and distributed embedded control, this is a great hands-on example.

---

## ✨ Features

- ✅ **Dual-MCU architecture** — clean separation of input and control logic
- ✅ **I2C communication** between Master and Slave microcontrollers
- ✅ **Keypad input** for real-time speed and position commands
- ✅ **LCD display** for live system feedback
- ✅ Modular codebase — `MCU1_Master` and `MCU2_Slave` are independently maintained
- ✅ Easily extendable to more slave nodes on the I2C bus

---

## 🏗️ System Architecture

```
┌─────────────────────────┐         I2C Bus        ┌──────────────────────────┐
│      MCU1 — Master      │ ──────────────────────→ │      MCU2 — Slave        │
│                         │                         │                          │
│  [Keypad Input]         │    Sends speed/position │  [Motor Driver - L293D]  │
│  [LCD Display]          │    commands over I2C    │  [DC Motor]              │
│  [User Interface Logic] │                         │  [Motor Control Logic]   │
└─────────────────────────┘                         └──────────────────────────┘
```

**Master (MCU1):** Handles all user interaction — reads keypad inputs, displays status on the LCD, and transmits commands to the slave over I2C.

**Slave (MCU2):** Receives commands from the master and directly drives the DC motor — handling PWM speed control and directional logic via the L293D motor driver.

---

## 🔧 Hardware Requirements

| Component | Quantity | Details |
|-----------|----------|---------|
| Arduino Uno / Nano | 2 | One master, one slave |
| L293D Motor Driver IC | 1 | For motor interfacing |
| 4x4 Keypad | 1 | User speed/position input |
| 16x2 LCD Display | 1 | System status feedback |
| DC Motor | 1 | 5V–12V |
| I2C Pull-up Resistors | 2 | 4.7kΩ on SDA and SCL lines |
| Power Supply | 1 | Shared or separate for motor |

---

## 🚀 Getting Started

### 1. Clone the repository
```bash
git clone https://github.com/my-ke20/Advanced-Motor-control.git
cd Advanced-Motor-control
```

### 2. Flash each MCU separately

Open **`MCU1_Master/`** in Arduino IDE → upload to your **first Arduino** (Master).

Open **`MCU2_Slave/`** in Arduino IDE → upload to your **second Arduino** (Slave).

### 3. Wire the I2C bus
Connect the **SDA** and **SCL** pins of both Arduinos together, with **4.7kΩ pull-up resistors** to 5V on each line.

### 4. Connect peripherals
- Keypad and LCD → Master Arduino
- L293D and DC Motor → Slave Arduino

### 5. Power up and test
Type a speed or position value on the keypad — the slave will respond and the motor will move accordingly.

---

## 📁 Project Structure

```
Advanced-Motor-control/
├── MCU1_Master/            # Master Arduino firmware (keypad + LCD + I2C sender)
│   └── MCU1_Master.ino
├── MCU2_Slave/             # Slave Arduino firmware (I2C receiver + motor driver)
│   └── MCU2_Slave.ino
├── MCU1_Master_code        # Raw code reference
├── MCU2_Slave_code         # Raw code reference
├── Screenshot (5).png      # Circuit/simulation screenshot
└── README.md
```

---

## 🧠 Key Concepts Demonstrated

- **I2C Protocol** — master-slave communication between two microcontrollers
- **PWM Motor Control** — variable speed via duty cycle
- **Modular Embedded Design** — separation of UI logic from control logic
- **Real-time Input Handling** — keypad scanning and debouncing

---

## 🔮 Possible Extensions

- Add more **slave nodes** on the I2C bus to control multiple motors
- Implement **PID closed-loop control** on the slave for precise positioning
- Add **UART/Bluetooth** on the master for wireless command input
- Log motor data to an **SD card** for analysis

---

## 👤 Author

**Michael Otieno Odhiambo**
- GitHub: [@my-ke20](https://github.com/my-ke20)
- LinkedIn: [Michael Otieno](https://www.linkedin.com/in/michael-otieno-8a6a8a35b/)
- Email: michaelotieno915@gmail.com

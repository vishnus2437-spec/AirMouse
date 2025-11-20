#  ESP32 AirMouse using MPU6050

A wireless motion-controlled mouse built with **ESP32** and **MPU6050** that lets you move your computer cursor by simply moving your hand in the air. The ESP32 acts as a Bluetooth HID mouse, while the MPU6050 tracks motion using its gyroscope and accelerometer. This project is customised to extract raw data from MPU6050 and process it so it even works well without any library dependencies.

---

##  Project Overview

ESP32 AirMouse converts natural hand movements into real-time cursor motion. With integrated buttons and smart gestures, it functions like a modern wireless mouse — but without needing a physical surface.

This project demonstrates sensor fusion, Bluetooth HID communication, and embedded gesture recognition in a compact DIY system.

---

##  Features

- ✅ Wireless Bluetooth Mouse (HID)
- ✅ Real-time gyro-based cursor control
- ✅ Left Click & Right Click
- ✅ Double Click
- ✅ Click & Drag
- ✅ Shake-to-Recenter Gesture
- ✅ Pause / Resume Motion Button
- ✅ LED Status Indicator
- ✅ Adjustable cursor sensitivity

---

##  Hardware Required

- ESP32 DevKit V1
- MPU6050 Gyroscope & Accelerometer Module
- 3 × Tactile Push Buttons
- Breadboard
- Jumper Wires
- USB Cable

---

##  Wiring Connections

### ESP32 → MPU6050

| ESP32 Pin | MPU6050 Pin |
| --------- | ----------- |
| 3.3V      | VCC         |
| GND       | GND         |
| GPIO 21   | SDA         |
| GPIO 22   | SCL         |

### Buttons

| Function      | ESP32 Pin |
| ------------- | --------- |
| Left Click    | GPIO 19   |
| Right Click   | GPIO 18   |
| Pause / Reset | GPIO 15   |

---

##  How It Works

The MPU6050 continuously measures angular velocity and acceleration. The ESP32 reads this data via I2C and maps it into 2D cursor movement. The BleMouse library allows the ESP32 to behave like a native Bluetooth mouse.

### Gesture Logic:

- Hand movement → Cursor movement
- Button press → Mouse click
- Fast shake → Cursor recenter
- Long hold → Drag
- Double tap → Double click

---

##  Installation & Setup

1. Install required Arduino libraries:

   - Adafruit MPU6050
   - Adafruit Unified Sensor
   - ESP32 BLE Mouse

2. Open the `airmouse.ino` file in Arduino IDE.

3. Select board: **ESP32 Dev Module**

4. Select correct COM Port

5. Upload code to ESP32

6. Pair ESP32 via Bluetooth as a mouse

7. Start controlling the cursor by moving the device



##  Applications

- Presentation pointer
- Air gesture controller
- VR / AR navigation
- Smart TV remote
- Accessibility interface
- DIY motion control systems

---

##  Customization

You can modify parameters in the code:

- Cursor speed
- Shake sensitivity
- Double-click delay
- Deadzone threshold

---

##  Troubleshooting

| Issue                 | Possible Cause                      |
| --------------------- | ----------------------------------- |
| MPU6050 not detected  | Loose wires / wrong I2C pins        |
| Cursor drifting       | Disable motion via reset button     |
| Buttons not working   | Wrong breadboard row or orientation |
| Bluetooth not visible | Reboot ESP32 and re-pair            |

---

##  Future Enhancements

- Scroll control using tilt
- Volume control gestures
- Haptic feedback
- Profile switching
- 3D air navigation

---

##  Author

Developed by **Vishnu S**

Student, Electronics & Communications, UVCE\
ESP32 Motion-Controlled AirMouse Project

##


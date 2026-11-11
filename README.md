# Multi-Tasking Robot

A versatile, multifunctional robotics system that combines autonomous navigation, remote control capabilities and automated waste management into a single mobile platform. Controlled via a custom mobile application, this project integrates dynamic obstacle avoidance, line tracking, Bluetooth teleoperation, and a smart automated bin.

## 📌 Features Overview

* **🚗 Obstacle Avoiding Car:** Autonomously navigates dynamic environments using an ultrasonic sensor to detect objects and automatically re-route its path.

* **🛣️ Line Follower Robot:** Uses Infrared (IR) contrast sensors to detect and track predefined ground paths, ideal for structured routes like warehouse automation.

* **📱 Bluetooth Control System:** Allows real-time remote switching between autonomous modes and manual control via a custom mobile app using an HC-05 Bluetooth module.

* **🗑️ Smart Automated Bin:** Automatically opens and closes its motorized lid using ultrasonic proximity detection and alerts users when full via an integrated LCD display.

## 🛠️ System Architecture & Components

### Hardware Components

* **Microcontroller:** Arduino / Microcontroller unit (main logic controller)

* **Distance Sensor:** Ultrasonic Sensor (HC-SR04 or equivalent) for obstacle detection and bin proximity

* **Line Sensors:** Infrared (IR) Sensor Array for surface contrast tracking

* **Wireless Module:** HC-05 Bluetooth Module for smartphone connectivity

* **Actuators:**

  * DC Gear Motors + Motor Driver (L298N/L293D) for locomotion

  * Servo Motor for the Smart Bin lid mechanism

* **Display & Feedback:** LCD Display for smart bin status monitoring

* **Power Supply:** Rechargeable Li-ion battery pack / Power source

### Software & Control

* **Mobile App:** Custom Android/iOS application for toggling operating modes and manual control.

* **Firmware:** Microcontroller C/C++ embedded code.

## ⚙️ How It Works

### 1. Obstacle Avoiding Mode

The front-mounted ultrasonic sensor continuously measures distance to ahead objects. When an obstacle is detected within a threshold distance, the microcontroller calculates an alternative path and signals the motor driver to steer the robot safely away.

### 2. Line Follower Mode

The IR sensor array monitors light reflectance on the surface to distinguish between a dark line and a light surface (or vice versa). When the robot veers off-course, differential feedback adjusts individual motor speeds to keep the robot aligned with the track.

### 3. Bluetooth Remote Control Mode

The HC-05 Bluetooth module receives wireless commands from the mobile app. Users can manually steer the robot (Forward, Backward, Left, Right) or switch between different operational modes on the fly.

### 4. Smart Bin Automation

An ultrasonic sensor detects when an object or trash is brought near the bin opening, triggering a servo motor to open the lid automatically. After a predetermined delay, the lid closes. An LCD screen alerts users when the bin is full and requires emptying.

## 🚀 Getting Started

### Prerequisites

* [Arduino IDE](https://www.arduino.cc/en/software) (or your preferred microcontroller IDE)

* Custom Mobile App APK (or Bluetooth Controller App)

### Installation & Setup

1. **Hardware Assembly:**

   * Connect the motor driver to the microcontroller pins according to the pin configuration map.

   * Mount the IR array on the bottom-front chassis and the ultrasonic sensor on the top-front.

   * Connect the HC-05 module to the hardware/software serial pins.

   * Attach the servo motor and secondary ultrasonic sensor to the Smart Bin mechanism.

2. **Uploading Firmware:**

   * Clone this repository:

     ```
     git clone https://github.com/soumit02/Multi-Tasking-Robot.git
     
     ```

   * Open `Multi_Tasking_Robot.ino` in Arduino IDE.

   * Select your board and port, then click **Upload**.

3. **Mobile App Pairing:**

   * Enable Bluetooth on your smartphone and pair with `HC-05` (Default PIN: `1234` or `0000`).

   * Launch the companion mobile app and establish a connection.

   * Select your desired mode from the application interface.


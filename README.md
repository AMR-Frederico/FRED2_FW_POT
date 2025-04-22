# 🚀 FRED2 Motor Controller Firmware
Firmware for controlling the front and back motors of the FRED2 robot using micro-ROS on ESP32 microcontrollers, with real-time PID control, encoder feedback, and ROS 2 communication.

## 📜 Overview
This firmware runs on two separate ESP32 boards:

- One controls the front motors.

- Another controls the back motors.

Each ESP32 reads motor encoder data, processes it, computes the necessary motor commands via robot kinematics and PID controllers, and sends commands to the motors.
Both ESP32s communicate with a ROS 2 system via micro-ROS to receive /cmd_vel velocity commands and publish motor telemetry.

## ⚙️ Features

✅ micro-ROS Integration: ROS 2 topics for commands and feedback.

✅ Encoder Reading: High-precision feedback via AS5600 encoders.

✅ PID Control: Per-wheel closed-loop motor control.

✅ Median Filtering: (Optional) Encoder signal smoothing.

✅ Robot Kinematics: Calculates wheel velocities from robot motion.

✅ Timeout Protection: Automatically stops motors if no commands received.

✅ Separate Environments: Independent firmware for front and back motors.

## 🛠 Project Structure

| File/Folder  |  Purpose |
|--------------|----------|
|main.cpp               |	Main firmware loop and initialization
|micro_ros.h/.cpp       |	Handles ROS node, publishers, subscribers
|encoder.h/.cpp         |	Read encoder data
|MedianFilter.h/.cpp    |	Remove outliers from sensor data
|kinematics.h/.cpp      |	Forward/inverse robot kinematics calculations
|pid_controller.h/.cpp  |	PID controller logic for wheels
|power.h/.cpp           |	Send PWM signals to motor drivers


## 🧩 Dependencies
- ESP32 Arduino Core
- micro-ROS Arduino library
- rclc, rcl, and micro-ROS client libraries
- Standard Arduino libraries


## 🌎 Environment Setup

|Environment    | Node Name	     | Namespace    | Purpose
|---------------|--------------  |--------------|---------|
|Back Motors    |fred2_fw_motors |	back	    | Controls rear motor pair
|Front Motors	|fred2_fw_motors |	front	    | Controls front motor pair

Each micro-ROS node uses a different namespace to avoid ROS topic collisions.

## 📡 ROS Topics
Both environments publish and subscribe to:


| Topic   	                 |    Message Type    |	Description    |
|----------------------------|--------------------|----------------|
|/cmd_vel/safe	             | geometry_msgs/Twist|	(Subscriber) Receive safe velocity commands
|/power/encoder/ticks/left	 | std_msgs/Int32	  | (Publisher) Left encoder ticks
|/power/encoder/ticks/right	 | std_msgs/Int32	  | (Publisher) Right encoder ticks
|/power/controlled/pwm/left	 | std_msgs/Float32	  | (Publisher) Left motor PWM signal
|/power/controlled/pwm/right |	std_msgs/Float32  |	(Publisher) Right motor PWM signal


## 🧠 Control Loop Overview
1. Receive /cmd_vel from ROS 2 system.
2. Compute wheel velocities based on robot kinematics.
3. Convert velocities from m/s ➔ rad/s ➔ RPM ➔ PWM.
4. Read actual RPM from wheel encoders.
5. PID controllers compute corrections.
6. Send PWM signals to motor drivers.
7. Publish telemetry (encoder ticks, PWM) back to ROS 2.

## 🧹 Code Style
- Doxygen-style comments (@brief, @param, @return)
- Structured sections (// -------------------------------------------------------)
- Minimalist and efficient C++ code for embedded systems
- Clear separation between logic (PID, kinematics) and hardware access (encoder reading, PWM writing)

## 🔧 How to Build and Flash
- Install ESP32 bo    ard support in PlatformIO.
- Install micro-ROS Arduino library.
- Select the appropriate board (e.g., ESP32 Dev Module).
- Build and flash main.cpp to the ESP32 controlling either the front or back motors.
- Configure micro-ROS agent on the ROS 2 PC to bridge serial communication.


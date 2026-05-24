# Dual-Mode Ultrasonic Radar System: STM32 & MATLAB

## Overview
A complete mechatronics project bridging embedded hardware and real-time software visualization. This system utilizes an STM32F103C8T6 microcontroller to drive a 180-degree radar sweep using an HC-SR04 ultrasonic sensor and a servo motor. 

The project highlights two major architectural features:
1. **Dual-Control Operation:** Hardware interrupts allow the user to instantly toggle between an automated environmental scanning mode and a manual analog joystick override.
2. **Dual-Transmission Method:** The system allows on-the-fly toggling between wired USB-to-TTL and wireless HC-06 Bluetooth serial communication. It streams rich data packets—including servo pulse width (angle), calculated distance, and raw echo time—directly to a live MATLAB polar plot.

## Hardware Architecture
*(Replace this line with your image: `![Project Schematic](Radar_Project_Schematic.png)`)*

**Components Used:**
* **Microcontroller:** STM32F103C8T6 (Blue Pill)
* **Sensor:** HC-SR04 Ultrasonic Distance Sensor
* **Actuator:** 180° Servo Motor
* **Communication:** HC-06 Bluetooth Module (Wireless UART) & USB-to-TTL (Wired UART)
* **Inputs:** Analog Joystick (ADC) & Push-Buttons (EXTI / Hardware Interrupts)

## Key Engineering Features
* **Decoupled MATLAB Architecture:** Solved UART blocking and timeout crashes by engineering a custom, non-blocking port-listening function. The MATLAB script simultaneously monitors both COM ports (USB at 115200 baud, Bluetooth at 9600 baud), dynamically flushes ghost data, and seamlessly switches between wired and wireless transmission without freezing the visualizer.
* **Robust Data Pipeline:** Instead of just sending simple distance metrics, the STM32 packages and transmits the exact PWM pulse width, distance in cm, and raw timer ticks. This allows MATLAB to calculate and plot the beam's true geometric position in real-time.
* **Dynamic Step Scaling:** Implemented asymmetric sensitivity math in C to ensure smooth, relative servo movement regardless of the physical joystick's resting voltage offsets.

## How to Run the Visualization
1. Flash the `main.c` code to the STM32 via STM32CubeIDE.
2. Wire the hardware according to the provided schematic.
3. Open the MATLAB script. Update `portName1` and `portName2` to match your active COM ports.
4. Run the script and use the UI toggle button to initiate the connection and start drawing the live polar plot.

## About the Developer
I am a Mechatronics Engineering student and a dedicated robotics enthusiast. This project was built to put theoretical coursework and software training into practical application, exploring my passion for embedded systems, hardware-software integration, and dynamic control logic.

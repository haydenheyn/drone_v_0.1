# ESP32 Drone Flight Controller

A real-time flight controller built from scratch using the ESP32 microcontroller. This project implements sensor fusion, PID control, and real-time task management to stabilize a quadcopter using IMU data and output motor control signals.

## 🚀 Project Overview

This flight controller is designed for a custom-built drone using:
- **ESP32** as the core processor
- **MPU6050** for accelerometer and gyroscope data
- **FreeRTOS** for real-time multitasking
- **Madgwick Filter** for sensor fusion
- **PID Control Loops** for flight stabilization
- **PWM Motor Control** for thrust regulation

## 🧠 Features

- Real-time orientation estimation (pitch, roll, yaw) using the Madgwick filter
- Sensor data acquisition at 1 kHz from MPU6050
- Tunable multi-axis PID control system
- Modular FreeRTOS task structure (IMU, Control, Output, Debug)
- UART serial telemetry for real-time monitoring
- Customizable architecture for future expansion (altitude hold, GPS, etc.)

## 📦 Architecture


## ⚙️ Tech Stack

- **Platform:** ESP-IDF (C++), FreeRTOS
- **MCU:** ESP32
- **Sensor:** MPU6050 (I²C)
- **Filter:** Madgwick AHRS Algorithm
- **Control:** PID Loops (Roll, Pitch, Yaw)

## 📷 Demo

## PID

## Electrical

## Setup

## Flash instructions

## 📁 File Structure



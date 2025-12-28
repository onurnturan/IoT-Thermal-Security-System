# IoT-Thermal-Security-System
**Overview**
This project is an advanced Embedded IoT Application developed on the Nuvoton M251 (Cortex-M23) microcontroller. It integrates a TPIS 1S 1385 Thermopile Sensor and a Quectel LTE Module to perform non-contact temperature measurement, occupancy detection, and remote alerting via MQTT and SMS.The system is designed for security and environmental monitoring, featuring autonomous logic to detect human presence based on thermal signatures and report data to a cloud broker.
**Key Features**
Thermal Sensing: Non-contact object and ambient temperature calculation using I2C protocol.LTE Connectivity: Full integration with Quectel LTE modules using AT commands.Cloud Communication: Telemetry data transmission via MQTT (test.mosquitto.org).Smart Alerting: Automatic SMS notification when a temperature threshold is exceeded (Intruder detection).Remote Configuration: Dynamic adjustment of temperature thresholds through incoming SMS commands.Efficient Memory Management: Circular buffer implementation for interrupt-driven UART communication.
**Hardware Stack**
MCU: Nuvoton M251 series (Cortex-M23 architecture).
Sensor: TPIS 1S 1385 (Thermopile) via I2C.
Module: Quectel LTE (EC25 series) via UART.
Debug/Terminal: PC interfacing via RS232 (UART0).
**System Flow**
Initialization: Clock, I2C, and dual UART peripherals are configured.
Calibration: Sensor constants are read from the sensor's EEPROM.
LTE/MQTT Setup: The LTE module connects to the network, opens an MQTT client, and subscribes to command topics.
Monitoring Loop:Read thermal data via I2C.Publish telemetry to MQTT.Check for presence (Threshold: >28.0°C).If presence detected: Send SMS Alert and MQTT Alarm.Poll for SMS to update the threshold dynamically.

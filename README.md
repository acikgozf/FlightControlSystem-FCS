# STM32 based Flight Control System
End-to-end designed flight control system that has without any RTOS middleware.

### Features
- Robust Gain-Scheduled PID control for Actuator and BLDC motor. I will try Robust Gain-Scheduled PID control in the future
- Support NMEA and UBX protocol for GNSS (NEO-M8N)
- 6 dof IMU (MPU-6050)
- RX/TX device EBYTE E01-2G4M27D is a DIP module based on originial imported nRF24L01P at 2.4Ghz with 500mW transmitting power
- Connection ability to GCS over USB Virtual Com Port
- Connection ability to GCS over lwIP(TCP/UDP)



# Plant Growth Control System

Arduino-based smart greenhouse controller that automatically manages temperature, humidity, soil moisture, and lighting for optimal plant growth.  
This version includes PID-driven temperature control, EEPROM-based soil calibration, and non-blocking relay logic for long-term stable operation.

---

## Features

### Air & Soil Monitoring
- Measures air temperature and humidity using a DHT11 sensor.
- Monitors soil temperature via a DS18B20 sensor.
- Measures soil moisture using multiple analog sensors with calibration.

### Automated Control
- PID-controlled heating for smooth air and soil temperature regulation.
- Fan-based ventilation to control air humidity and temperature.
- Automatic watering via soil moisture feedback.
- Mist generation for humidity maintenance.

### Lighting Control
- Automatically turns lighting ON/OFF based on RTC time (06:00–23:00).
- Supports custom LED or UV grow lights via relay output.

### Smart Logic
- EEPROM calibration retains soil sensor values between restarts.
- Non-blocking, time-proportional PID control avoids relay chatter.
- Deadband and thresholds prevent oscillation (“ping-pong” switching).
- Failsafe defaults ensure system keeps running even if a sensor fails.

### Modular & Scalable
- Add more soil sensors easily (averaging handled automatically).
- Expand with extra relays or actuators without code restructuring.

---

## Hardware Used

| Component | Purpose |
|-----------|---------|
| Arduino Uno (or compatible) | Main controller |
| DHT11 sensor | Measures air temperature & humidity |
| DS18B20 (OneWire) | Measures soil temperature |
| Analog soil moisture sensors (x2) | Measures soil water content |
| 12V water pumps (x2) | For soil irrigation & air misting |
| Fans (x2) | For air circulation & humidity reduction |
| Soil heating cable | Regulates soil temperature |
| PTC heaters / Hairdryer (1kW) | Regulates air temperature |
| Relay board (8-channel) | Switches high-power loads |
| RTC DS1307 module | Provides real-time clock for scheduling |
| 12V power supply | Powers all external devices |

---

## Software Requirements

- Arduino IDE (or compatible environment)
- Required libraries:
  - `DHT.h`
  - `OneWire.h`
  - `DallasTemperature.h`
  - `Wire.h`
  - `PID_v1.h`
  - `EEPROM.h`

---

## System Overview

### 1. Sensing
- Air: Temperature and humidity from DHT11.
- Soil: Temperature from DS18B20 and moisture from analog sensors.
- Multiple soil sensors are averaged for better reliability.

### 2. Control

| Parameter       | Actuator                  | Logic Type       | Control Strategy        |
|-----------------|---------------------------|-----------------|------------------------|
| Air Temperature | Air heater (relay)        | PID             | Time-proportional      |
| Soil Temperature| Soil heater (relay)       | PID             | Time-proportional      |
| Air Humidity    | Fans & mist pump          | Threshold       | ±5–10% hysteresis      |
| Soil Moisture   | Water pump                | Threshold       | 85% target moisture    |
| Light           | Custom LED/UV             | RTC-based       | 06:00–23:00 ON cycle   |

### 3. Automation Cycle
- All sensors are read every 2 seconds.
- PID computations run continuously with 10-second time windows.
- Relays are triggered using non-blocking proportional timing.
- EEPROM calibration ensures consistent soil moisture readings.

---

## Soil Moisture Calibration

Calibration is performed manually and stored in EEPROM:

1. Run calibration mode (via Serial command in future versions).
2. Place sensors dry, then fully wet, to record reference points.
3. System maps analog readings (0–1023) to 0–100% humidity.
4. Values are saved to EEPROM and automatically loaded at startup.

---

## PID Control

- Two independent PID loops:
  - Air PID: controls main air heater (and secondary PTC heaters).
  - Soil PID: controls soil heating cable.
- Output is scaled to a 10s time slice, giving smooth and safe relay activation.
- Prevents overshoot and minimizes temperature fluctuations.

---

## Safety & Reliability

- Relays use active LOW logic (fail-safe default OFF).
- Deadband of ±0.5°C prevents flicker.
- All control is non-blocking, no use of `delay()` for timing loops.
- Handles sensor read errors gracefully (uses fallback values).
- Ready for SSR or optocoupled relay boards for AC safety.

---


![Screenshot 2022-08-23 131313](https://user-images.githubusercontent.com/111133064/188284658-a9c3b89f-8e13-41a5-824c-6c36ba73c85d.png)
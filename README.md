Arduino Ventilation System
This Arduino project implements a ventilation control system using an L298N motor driver, a DHT11 sensor, an optical encoder for wind speed measurement, and a 16x2 LCD display. The system controls two fans based on environmental conditions:

Fan1: Relay-controlled mechanical fan, activated when wind speed (RPM) falls below a threshold.
Fan2: PWM-controlled fan via L298N, with speed proportional to temperature (0–40°C maps to 0–100% PWM).

Features

Measures wind speed using an optical encoder (RPM calculation).
Monitors temperature and humidity with a DHT11 sensor.
Controls Fan1 (relay) based on wind speed (< 20 RPM triggers ON).
Adjusts Fan2 speed (via L298N) based on temperature.
Displays fan statuses on a 16x2 LCD.
Logs data to Serial for debugging.

Hardware Requirements

Arduino board (e.g., Uno)
DHT11 temperature and humidity sensor
Optical encoder for wind speed measurement
L298N motor driver (for Fan2)
Relay module (for Fan1)
16x2 LCD display (with I2C adapter optional)
DC fan (for Fan2)
Mechanical fan (for Fan1)
Jumper wires, breadboard, and power supply

Pin Configuration



Component
Pin(s)



DHT11 Data
A0


Optical Encoder
2 (INT0)


Fan1 Relay
12


Fan2 L298N (IN3, IN4)
8, 7


Fan2 L298N PWM (ENB)
6


LCD (RS, EN, D4-D7)
13, 11, 10, 9, A2, A3


Dependencies
Install the following Arduino libraries:

LiquidCrystal: For 16x2 LCD control
DHT: For DHT11 sensor readings

Use the Arduino Library Manager to install them.
Installation

Clone this repository:git clone https://github.com/your-username/arduino-ventilation-system.git


Connect the hardware as per the pin configuration.
Install the required libraries via Arduino IDE.
Upload the code (ventilation_system.ino) to your Arduino board.
Open the Serial Monitor (9600 baud) to view logs.

Usage

The system initializes and displays "Vent Sys Init" on the LCD.
Every 2 seconds, it:
Calculates RPM from the optical encoder.
Reads temperature and humidity from DHT11.
Turns Fan1 ON if RPM < 20, else OFF.
Sets Fan2 speed (PWM) proportional to temperature (0–40°C → 0–100%).
Updates the LCD with Fan1 (ON/OFF) and Fan2 (PWM %) statuses.
Logs data to Serial Monitor.



Notes

Debouncing: The optical encoder uses a 20ms debounce to avoid false pulses.
PWM Control: Fan2 speed is mapped from temperature (0–40°C) to PWM (0–255).
Error Handling: DHT11 read failures are logged to Serial.
LCD: Ensure pin assignments avoid conflicts with other components.

License
MIT License. See LICENSE for details.


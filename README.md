# ESP32 Pump controller project

This project uses an ESP32 microcontroller to read distance measurements from an ultrasonic sensor, control LEDs, and manage relay states based on distance thresholds set via an encoder with a button. The lower and upper limits are stored in EEPROM for persistence and are loaded when the system starts.

## Features

- **Ultrasonic Distance Sensing**: Measures the distance using an ultrasonic sensor connected to the ESP32.
- **Encoder with Button**: Allows the user to set lower and upper distance limits.
- **EEPROM Storage**: Saves and retrieves lower and upper limits from EEPROM.
- **LED Indicators**: Provides visual feedback for power, lower limit, upper limit, and relay state.
- **Relay Control**: Activates a relay based on the measured distance.

## Components

- ESP32 DevKitC
- Ultrasonic sensor (connected to pins 18 and 19)
- Encoder with button (connected to pins 22, 23, and 21)
- LEDs for indicators (connected to pins 14, 15, 2, and 16)
- Relay (connected to pin 12)
- EEPROM for storing configuration

## Connections
![Pinout picture](/assets/pinout.png)
[Schematics](/assets/ServoBlinds1.0_schematics_modded_for_pumpcontroller.pdf)

- No need to use extra resistors with the LEDs. Those are on the PCB (82ohm).


## Usage

1. **Initial Setup**: On the first run, set the lower and upper limits using the encoder and save them with the button.
2. **Teach new values**: Hold the encoder button for 5 seconds to reset the EEPROM values. LEDs will blink to notify the reset. After clearing the values, new ones need to be set. This is done as follows:

    - *Set the low level treshold.*: Make sure the tank is as empty as it is allowed to be. Turn the encoder clockwise until a green LED is turned on. Each encoder click is 20mm so if the level you are looking for is 1m from the ultrasonic sensor, 50 tics are required.
    - *Press the encoder shaft button to save the value.*
    - *Set the upper lever treshold.*: Make sure the tank is as full as allowed. Turn the encoder clockwise until a yellow LED is turned on.
    - *Press the encoder shaft button to save the value.*

    - Values are now saved and loaded in next restart.

3. **Normal Operation**: The system will control the relay based on the distance measured and the configured limits. Once the lower limit is reached, the relay will turn on. The relay will stay on as long as the lower limit is reached.
The values is only checked once a second to not blink the relay too fast.


## Functions

### InitialSetup
Sets the lower and upper limits using the encoder and saves them to EEPROM.

### blinkLEDs
Blinks all LEDs three times to indicate certain events (like resetting EEPROM).

### resetEEPROM
Resets the EEPROM values for the lower and upper limits.

### breatheLED
Creates a breathing effect for the power LED to indicate the system is running.

## Library Files

- `UltrasonicSensor.h` and `UltrasonicSensor.cpp`: Handles ultrasonic sensor readings. The library is based on this one by [ritesht](https://github.com/riteshRcH/waterproof_small_blind_ultrasonic_sensor_DYP-A02YYxx_v1.0/blob/master/code/arduino/DYPA02YYUM_v1_arduino_uno_code/DYPA02YYUM_v1_arduino_uno_code.ino)
- `KnobManager.h` and `KnobManager.cpp`: Manages the encoder and button input.

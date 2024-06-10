//Based on: https://github.com/riteshRcH/waterproof_small_blind_ultrasonic_sensor_DYP-A02YYxx_v1.0/blob/master/code/arduino/DYPA02YYUM_v1_arduino_uno_code/DYPA02YYUM_v1_arduino_uno_code.ino

#ifndef ULTRASONICSENSOR_H
#define ULTRASONICSENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
public:
    UltrasonicSensor(uint8_t rxPin, uint8_t txPin, uint32_t baudRate = 9600);
    void begin();
    unsigned int getDistance();

private:
    HardwareSerial* serial;
    uint8_t rxPin;
    uint8_t txPin;
    uint32_t baudRate;
};

#endif

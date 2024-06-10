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

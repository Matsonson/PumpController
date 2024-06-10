#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>

class Knob;

class Knob{
public:
    Knob(int,int,int);
    ESP32Encoder encoder;

    int getRange();
    void setRange(int range);
    bool buttonPressed();
    int64_t getCount();


private:
    int minVal;
    int maxVal;
    int _btnPin;

};

#pragma once
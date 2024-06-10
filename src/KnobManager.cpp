#include <Arduino.h>
#include <KnobManager.h>

Knob::Knob(int pinA,int pinB,int btnPin){
    
    _btnPin=btnPin;
    pinMode(_btnPin, INPUT_PULLUP);
    encoder.attachHalfQuad(pinA,pinB);    
    minVal=0;
    maxVal=100;

}

int Knob::getRange(){
    return maxVal-minVal;
}

void Knob::setRange(int range){
    maxVal=range;
}

int64_t Knob::getCount(){
    //Limit the count between the min and max value
    if (encoder.getCount()>maxVal)
    {
        encoder.setCount(maxVal);
    }
    else if (encoder.getCount()<minVal)
    {
        encoder.setCount(minVal);
    }
    return encoder.getCount();
}

bool Knob::buttonPressed(){
    return !digitalRead(_btnPin);
}


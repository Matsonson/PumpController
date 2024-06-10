#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t rxPin, uint8_t txPin, uint32_t baudRate)
    : rxPin(rxPin), txPin(txPin), baudRate(baudRate) {
    serial = &Serial2;
}

void UltrasonicSensor::begin() {
    serial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

unsigned int UltrasonicSensor::getDistance() {
    byte hdr, data_h, data_l, chksum;
    unsigned int distance = 0;

    // Read from the serial port if data is available
    while (serial->available()) {
        hdr = (byte)serial->read();
        if (hdr == 255) {
            data_h = (byte)serial->read();
            data_l = (byte)serial->read();
            chksum = (byte)serial->read();

            if (chksum == ((hdr + data_h + data_l) & 0x00FF)) {
                distance = data_h * 256 + data_l;
                break; // Exit the while loop once a valid reading is obtained
            }
        }
    }

    // Flush the serial buffer to remove any remaining old data
    while (serial->available()) {
        serial->read();
    }

    return distance;
}
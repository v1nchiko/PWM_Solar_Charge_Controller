//
// Created by Ivan Bondaruk on 11/6/2023.
//

#include <Arduino.h>
#include "VoltageSensor.h"

VoltageSensor::VoltageSensor(VoltageSensorType sensorType, uint8_t _pin)
{
    pin = _pin;
    Value = 0;

    // Different models of the sensor have their sensitivity:
    switch (sensorType) {
        case VOLTSENS_20K:
            R1 = 20000.0;
            R2 = 10000.0;
            break;
        case VOLTSENS_50K:
            R1 = 50000.0;
            R2 = 10000.0;
            break;
        case VOLTSENS_100K:
            R1 = 100000.0;
            R2 = 10000.0;
            break;
        case VOLTSENS_200K:
            R1 = 200000.0;
            R2 = 10000.0;
            break;
    }
}

void VoltageSensor::ReadValue()
{
    int analogValue = analogRead(pin); // Get value from analog pin
    double temp = (analogValue * 5.0) / 1024.0; // Converting voltage value
    Value = temp / (R2/(R1 + R2));
}

double VoltageSensor::GetValue() const {
    return Value;
}

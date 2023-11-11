//
// Created by Ivan Bondaruk on 11/6/2023.
//

#ifndef PWMSCC_VOLTAGESENSOR_H
#define PWMSCC_VOLTAGESENSOR_H

#include <stdint.h>

enum VoltageSensorType {VOLTSENS_20K, VOLTSENS_50K, VOLTSENS_100K, VOLTSENS_200K};

class VoltageSensor {
public:
    VoltageSensor(VoltageSensorType sensorType, uint8_t _pin); // constructor
    double GetValue() const;
private:
    uint8_t pin;
    double R1,R2;
};


#endif //PWMSCC_VOLTAGESENSOR_H

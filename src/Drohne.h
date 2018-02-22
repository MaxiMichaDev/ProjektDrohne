//
// Created by Micha on 21.02.2018.
//

#ifndef PROJEKTDROHNE_DROHNE_H
#define PROJEKTDROHNE_DROHNE_H

#include "Arduino.h"

bool isRemoteControlled();

#define FILE_BASE_NAME "log"

struct data_t {
    uint32_t time;
    float gyro[3];
    float servoPercentage[2];
};
void setupDrone();

void mainLoop(data_t *data);

void switchedToRemoteControl();

void printData(Print* pr, data_t* data);
void printHeader(Print* pr);

#endif //PROJEKTDROHNE_DROHNE_H
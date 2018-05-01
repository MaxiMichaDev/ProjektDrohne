//
// Created by Micha on 10.02.2018.
//

#include <Servo.h>
#include <math.h>
#include "TiltController.h"
#include <Arduino.h>


TiltController::TiltController(unsigned int servoCount, Servo *servo, const int *sign, uint8_t degreeZero,
                               uint8_t degreeRange,
                               uint8_t maxAdjust) :
        servoCount(servoCount), servo(servo), sign(sign), degreeZero(degreeZero), degreeRange(degreeRange),
        maxAdjust(maxAdjust) {

}

void TiltController::supply(float gyroValue) {
    const float delta = target - gyroValue * 180 / M_PI;

    auto adjust = static_cast<int>(floor(rawAdjust));

    for (int i = 0; i < servoCount; ++i) {
        servoValue = limitDegree(static_cast<uint8_t>(degreeZero + sign[i] * (getAmplitudePercentage(delta) / 100 * degreeRange + adjust)));
        servo[i].write(servoValue);
    }

    rawAdjust += /*f(delta)*/ static_cast<float>(I_Factor) / 100 * sgn(delta);
    rawAdjust = constrain(rawAdjust, -maxAdjust, maxAdjust);
}

void TiltController::targetDegree(float target) {
    TiltController::target = target;
}

uint8_t TiltController::limitDegree(uint8_t degree) const {
    return constrain(degree, degreeZero - degreeRange, degreeZero + degreeRange);
}

float TiltController::getAmplitudePercentage(float degreeDifference) {
    return fabs(degreeDifference * 6) < 100 ? degreeDifference * 6 : 100 * sgn(degreeDifference);
    //return (1 / (1 + exp(-5 * degreeDifference / 100)) - 0.5) * 200;
}

float TiltController::f(float x) {
    return I_Factor;
    //return (2.8 / (1 + exp(-0.5*(fabs(x)))) - 1.3);
    //return (abs(x) < 50) ? (0.1 * abs(x) + 0.2) : 1;
}

int TiltController::sgn(float val) {
    return (val > 0) - (val < 0);
}

uint8_t TiltController::getServoValue() {
    return servoValue;
}

void TiltController::reset() {
    rawAdjust = 0;
}



void TiltController::changeI_Factor(uint8_t i) {
    I_Factor = static_cast<uint8_t>(i < 35 ? 0 : i);
    if (I_Factor == 0) {
        reset();
    }
}


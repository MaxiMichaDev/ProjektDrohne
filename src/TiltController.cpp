//
// Created by Micha on 10.02.2018.
//

#include <Servo.h>
#include <math.h>
#include "TiltController.h"

TiltController::TiltController(unsigned int servoCount, Servo *servo, const int *sign, uint8_t degreeZero,
                               uint8_t degreeRange,
                               uint8_t maxAdjust) :
        servoCount(servoCount), servo(servo), sign(sign), degreeZero(degreeZero), degreeRange(degreeRange),
        maxAdjust(maxAdjust) {

}

void TiltController::supply(float gyroValue) {

    for (int i = 0; i < servoCount; ++i) {
        servoValue = degreeZero - sign[i] * sgn(gyroValue) * degreeRange;
        servo[i].write(servoValue);
    }
}

void TiltController::targetDegree(float target) {
    TiltController::target = target;
}

//uint8_t TiltController::limitDegree(uint8_t degree) const {
//    if (degree > degreeZero + degreeRange) {
//        degree = degreeZero + degreeRange;
//    } else if (degree < degreeZero - degreeRange) {
//        degree = degreeZero - degreeRange;
//    }
//    return degree;
//}


int TiltController::sgn(float val) {
    return (val > 0) - (val < 0);
}

uint8_t TiltController::getServoValue() {
    return servoValue;
}
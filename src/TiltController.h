//
// Created by Micha on 10.02.2018.
//


#ifndef PROJEKTDROHNE_TILTCONTROLLER_H
#define PROJEKTDROHNE_TILTCONTROLLER_H

#endif //PROJEKTDROHNE_TILTCONTROLLER_H
class TiltController {
private:
public:
    TiltController(unsigned int servoCount, Servo *servo, const int *sign, float degreeZero, float degreeRange,
                   float maxAdjust);

    void supply(float gyroValue);
    void targetDegree(float target);

private:
    const unsigned int servoCount;
    Servo *servo;
    const int *sign;
    const float degreeZero;
    const float degreeRange;
    const float maxAdjust;

    float target = 0;
    float rawAdjust = 0;


    float limitDegree(float degree) const;
    float getAmplitudePercentage(float degreeDifference);
    float f(float x);
    int sgn(float val);
};



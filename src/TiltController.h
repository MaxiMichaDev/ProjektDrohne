//
// Created by Micha on 10.02.2018.
//


#ifndef PROJEKTDROHNE_TILTCONTROLLER_H
#define PROJEKTDROHNE_TILTCONTROLLER_H

class TiltController {
private:
public:
    TiltController(unsigned int servoCount, Servo *servo, const int *sign, uint8_t degreeZero, uint8_t degreeRange,
                   uint8_t maxAdjust);

    void supply(float gyroValue);
    void targetDegree(float target);
    uint8_t getServoValue();
    void reset();
    //   void changeI_Factor(int i);


private:
    const int servoCount;
    Servo *servo;
    const int *sign;
    const uint8_t degreeZero;
    const uint8_t degreeRange;
    const uint8_t maxAdjust;

    uint8_t servoValue;
    float target = 0;
    float rawAdjust = 0;
    //   float I_Factor = 0;


    uint8_t limitDegree(uint8_t degree) const;
    float getAmplitudePercentage(float degreeDifference);
    //   float f(float x);

    int sgn(float val);

};

#endif //PROJEKTDROHNE_TILTCONTROLLER_H



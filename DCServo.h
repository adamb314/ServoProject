#include <Arduino.h>
#include <Adafruit_DotStar.h>
#undef max
#undef min
#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"

#include <Eigen.h>
#include "CurrentControlLoop.h"
#include "EncoderHandler.h"
#include "KalmanFilter.h"

#ifndef DC_SERVO_H
#define DC_SERVO_H

class DCServo
{
 public:
    static DCServo* getInstance();

    void enable(bool b = true);

    void setReference(int16_t pos, int16_t vel, int16_t feedForwardU = 0);

    int16_t getPosition();

    int16_t getVelocity();
    
    int16_t getControlSignal();

    int16_t getCurrent();

    uint16_t getLoopNumber();

 private:
    DCServo();

    void controlLoop();

    int16_t setOutput(int16_t u);

    bool controlEnabled;

    Eigen::Vector3f L;

    uint16_t loopNumber;
    int16_t rawPos;
    Eigen::Vector3f x;

#ifdef SIMULATE
    Eigen::Vector3f xSim;
#endif

    int16_t current;
    int16_t controlSignal;
    int16_t feedForwardU;
    int16_t velRef;
    int16_t posRef;

    float Ivel;

    std::unique_ptr<CurrentControlLoop> currentControl;
    std::unique_ptr<EncoderHandler> encoderHandler;
    std::unique_ptr<KalmanFilter> kalmanFilter;

    Adafruit_DotStar dotStarLed;

    int16_t pwmOutputOnDisabled;

    std::vector<FunctionThread*> threads;
};
#endif

#include "DummyTrajectoryGenerator.h"
#include <Eigen/Dense>

#include <numeric>
#include <cmath>

extern double pi;

#include <chrono>
#include <thread>
#include <algorithm>
#include <functional>
#include <mutex>

#include "ServoProject.h"

#ifndef ROBOT_H
#define ROBOT_H

class Robot
{
public:
    static const size_t dof = 6;

    std::unique_ptr<SimulateCommunication> communicationSim{std::make_unique<SimulateCommunication>()};

    Robot(Communication* communication, const std::array<bool, 7> simulate = {false}, double cycleTime = 0.018);

    virtual ~Robot();

    Eigen::Matrix<double, dof, 1> getPosition() const;

    void setHandlerFunctions(const std::function<void(double, Robot*)>& newSendCommandHandlerFunction,
            const std::function<void(double, Robot*)>& newReadResultHandlerFunction,
            const std::function<void(std::exception& e)>& newErrorHandlerFunction = [](std::exception& e){});

    void removeHandlerFunctions();

    void start();

    void shutdown();

    std::exception getUnhandledException();

    bool isAlive(bool raiseException = true);

    double getCycleSleepTime() const;

    struct Reference
    {
        Reference();
        Reference(Eigen::Matrix<double, dof, 1> pos, double gripperPos);
        Reference(TrajectoryItem<dof, double> trajItem, double gripperPos);

        TrajectoryItem<dof, double> trajItem;
        double gripperPos;
    };

    std::array<DCServoCommunicator*, dof> dcServoArray;
    DCServoCommunicator* gripperServo;

private:
    std::vector<std::unique_ptr<DCServoCommunicator> > initFunction(
        Communication* communication, const std::array<bool, 7> simulate);

    ServoManager servoManager;
};

Robot::Reference operator*(const Robot::Reference& a, float t);

Robot::Reference operator*(float t, const Robot::Reference& a);

Robot::Reference operator+(const Robot::Reference& a, const Robot::Reference& b);

Robot::Reference operator-(const Robot::Reference& a, const Robot::Reference& b);

Robot::Reference operator/(const Robot::Reference& a, float t);

template <class T>
T interpolate(const T& a, const T& b, float t)
{
    return a * (1 - t) + b * t;
}

template <class T>
class SamplingHandler
{
public:
    SamplingHandler(std::function<T()> inInc, double inputDt) :
            inputIncrementer(std::move(inInc)),
            inputDt(inputDt)
    {
        n = inputIncrementer();
        np1 = inputIncrementer();
    }

    void increment(float dt)
    {
        interpolT += dt;

        while (interpolT > inputDt)
        {
            interpolT -= inputDt;

            n = np1;
            np1 = inputIncrementer();
        }
    }

    T getSample()
    {
        return interpolate(n, np1, interpolT);
    }

private:
    std::function<T()> inputIncrementer;
    double inputDt;
    double interpolT{0};
    T n{};
    T np1{};
};

#endif

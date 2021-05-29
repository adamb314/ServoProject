#include "MasterCommunication.h"
#include "DCServoCommunicator.h"
#include <Eigen/Dense>

#include <numeric>
#define pi (M_PI)

#include <chrono>
#include <thread>
#include <algorithm>
#include <functional>
#include <mutex>

#ifndef ROBOT_H
#define ROBOT_H

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

class Robot
{
public:
    static const size_t dof = 6;

    std::unique_ptr<SimulateCommunication> communicationSim{std::make_unique<SimulateCommunication>()};

    Robot(Communication* communication, const std::array<bool, 7> simulate = {false}, double cycleTime = 0.018) :
            dcServoArray{{{1, simulate[0] ? communicationSim.get() : communication},
                            {2, simulate[1] ? communicationSim.get() : communication},
                            {3, simulate[2] ? communicationSim.get() : communication},
                            {4, simulate[3] ? communicationSim.get() : communication},
                            {5, simulate[4] ? communicationSim.get() : communication},
                            {6, simulate[5] ? communicationSim.get() : communication}}},
            gripperServo{7, simulate[6] ? communicationSim.get() : communication},
            cycleTime(cycleTime)
    {
        dcServoArray[0].setOffsetAndScaling(2 * pi / 4096.0, 0.950301, 0);
        dcServoArray[1].setOffsetAndScaling(2 * pi / 4096.0, -2.042107923, pi / 2);
        dcServoArray[2].setOffsetAndScaling(2 * pi / 4096.0, 1.0339, pi / 2);
        dcServoArray[3].setOffsetAndScaling(-2 * pi / 4096.0, -1.47531 + 0.242229, 0.0);
        dcServoArray[4].setOffsetAndScaling(2 * pi / 4096.0, 1.23394 + 0.393326, 0.0);
        dcServoArray[5].setOffsetAndScaling(-2 * pi / 4096.0, -1.45191 + 0.191361, 0.0);

        dcServoArray[0].setControlSpeed(25);
        dcServoArray[0].setBacklashControlSpeed(12, 3.0, 0.00);
        dcServoArray[0].setFrictionCompensation(0);
        dcServoArray[1].setControlSpeed(25);
        dcServoArray[1].setBacklashControlSpeed(12, 3.0, 0.00);
        dcServoArray[1].setFrictionCompensation(0);
        dcServoArray[2].setControlSpeed(25);
        dcServoArray[2].setBacklashControlSpeed(12, 3.0, 0.00);
        dcServoArray[2].setFrictionCompensation(0);
        dcServoArray[3].setControlSpeed(34);
        dcServoArray[3].setBacklashControlSpeed(14, 3.0, 0.0);
        dcServoArray[3].setFrictionCompensation(0);
        dcServoArray[4].setControlSpeed(34);
        dcServoArray[4].setBacklashControlSpeed(14, 3.0, 0.0);
        dcServoArray[4].setFrictionCompensation(0);
        dcServoArray[5].setControlSpeed(34);
        dcServoArray[5].setBacklashControlSpeed(14, 3.0, 0.0);
        dcServoArray[5].setFrictionCompensation(0);

        gripperServo.setOffsetAndScaling(pi / 1900.0, 0.0, 0.0);

        while (std::any_of(std::begin(dcServoArray), std::end(dcServoArray), [](auto& d)
                {
                    return !d.isInitComplete();
                }) || !gripperServo.isInitComplete())
        {
            std::for_each(std::begin(dcServoArray), std::end(dcServoArray), [](auto& d)
                {
                    d.run();
                });

            gripperServo.run();
        }

        std::transform(std::begin(dcServoArray), std::end(dcServoArray), std::begin(currentPosition), [](auto& d)
            {
                return d.getPosition();
            });

        for (size_t i = 0; i != 2; ++i)
        {
            gripperServo.setReference(pi / 2.0, 0.0, 0.0);
            gripperServo.run();
            gripperServo.getPosition();
        }

        t = std::thread{&Robot::run, this};
    }

    virtual ~Robot()
    {
        shutdown();
    }

    void run()
    {
        using namespace std::chrono;
        high_resolution_clock::time_point sleepUntilTimePoint = high_resolution_clock::now();
        high_resolution_clock::duration clockDurationCycleTime(
                duration_cast<high_resolution_clock::duration>(duration<double>(cycleTime)));

        while(!shuttingDown)
        {
            cycleSleepTime = std::chrono::duration<double>(sleepUntilTimePoint - high_resolution_clock::now()).count();
            std::this_thread::sleep_until(sleepUntilTimePoint);
            sleepUntilTimePoint += clockDurationCycleTime;

            std::function<void(double, Robot*)> tempSendHandlerFunction;
            std::function<void(double, Robot*)> tempReadHandlerFunction;
            {
                const std::lock_guard<std::mutex> lock(handlerFunctionMutex);
                tempSendHandlerFunction = sendCommandHandlerFunction;
                tempReadHandlerFunction = readResultHandlerFunction;
            }

            tempSendHandlerFunction(cycleTime, this);

            std::for_each(std::begin(dcServoArray), std::end(dcServoArray), [](auto& d)
                {
                    d.run();
                });

            gripperServo.run();

            std::transform(std::begin(dcServoArray), std::end(dcServoArray), std::begin(currentPosition), [](auto& d)
                {
                    return d.getPosition();
                });

            gripperServo.getPosition();
 
            tempReadHandlerFunction(cycleTime, this);
        }
    }

    Eigen::Matrix<double, dof, 1> getPosition()
    {
        return currentPosition;
    }

    void setHandlerFunctions(const std::function<void(double, Robot*)>& newSendCommandHandlerFunction,
            const std::function<void(double, Robot*)>& newReadResultHandlerFunction)
    {
        const std::lock_guard<std::mutex> lock(handlerFunctionMutex);

        sendCommandHandlerFunction = newSendCommandHandlerFunction;
        readResultHandlerFunction = newReadResultHandlerFunction;
    }

    void removeHandlerFunctions()
    {
        setHandlerFunctions([](double cycleTime, Robot* robot){}, [](double cycleTime, Robot* robot){});
    }

    void shutdown()
    {
        if (!shuttingDown)
        {
            shuttingDown = true;
            t.join();
        }
    }

    double getCycleSleepTime()
    {
        return cycleSleepTime;
    }

    std::array<DCServoCommunicator, dof> dcServoArray;
    DCServoCommunicator gripperServo;

private:
    double cycleTime;
    double cycleSleepTime{0};
    Eigen::Matrix<double, dof, 1> currentPosition{Eigen::Matrix<double, dof, 1>::Zero()};

    bool shuttingDown{false};

    std::thread t{};

    std::mutex handlerFunctionMutex{};
    std::function<void(double, Robot*)> sendCommandHandlerFunction{[](double cycleTime, Robot* robot){}};
    std::function<void(double, Robot*)> readResultHandlerFunction{[](double cycleTime, Robot* robot){}};
};

#endif

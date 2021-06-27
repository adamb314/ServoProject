#include "Robot.h"

Robot::Robot(Communication* communication, const std::array<bool, 7> simulate, double cycleTime) :
        dcServoArray{{{1, simulate[0] ? communicationSim.get() : communication},
                        {2, simulate[1] ? communicationSim.get() : communication},
                        {3, simulate[2] ? communicationSim.get() : communication},
                        {4, simulate[3] ? communicationSim.get() : communication},
                        {5, simulate[4] ? communicationSim.get() : communication},
                        {6, simulate[5] ? communicationSim.get() : communication}}},
        gripperServo{7, simulate[6] ? communicationSim.get() : communication},
        cycleTime(cycleTime)
{
    dcServoArray[0].setOffsetAndScaling(2 * pi / 4096.0, 0.832185, 0);
    dcServoArray[1].setOffsetAndScaling(2 * pi / 4096.0, -2.042107923, pi / 2);
    dcServoArray[2].setOffsetAndScaling(2 * pi / 4096.0, 1.0339, pi / 2);
    dcServoArray[3].setOffsetAndScaling(-2 * pi / 4096.0, -1.22728, 0.0);
    dcServoArray[4].setOffsetAndScaling(2 * pi / 4096.0, 0.8 * 1.42914 + 0.2 * 1.47876, 0.0);
    dcServoArray[5].setOffsetAndScaling(-2 * pi / 4096.0, -1.27661, 0.0);

    dcServoArray[0].setControlSpeed(25);
    dcServoArray[0].setBacklashControlSpeed(12, 3.0, 0.00);
    dcServoArray[0].setFrictionCompensation(0);
    dcServoArray[1].setControlSpeed(25);
    dcServoArray[1].setBacklashControlSpeed(12, 3.0, 0.00);
    dcServoArray[1].setFrictionCompensation(0);
    dcServoArray[2].setControlSpeed(25);
    dcServoArray[2].setBacklashControlSpeed(12, 3.0, 0.00);
    dcServoArray[2].setFrictionCompensation(0);
    dcServoArray[3].setControlSpeed(30);
    dcServoArray[3].setBacklashControlSpeed(12, 3.0, 0.0);
    dcServoArray[3].setFrictionCompensation(0);
    dcServoArray[4].setControlSpeed(30);
    dcServoArray[4].setBacklashControlSpeed(12, 3.0, 0.0);
    dcServoArray[4].setFrictionCompensation(0);
    dcServoArray[5].setControlSpeed(30);
    dcServoArray[5].setBacklashControlSpeed(12, 3.0, 0.0);
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

Robot::~Robot()
{
    shutdown();
}

void Robot::run()
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

Eigen::Matrix<double, Robot::dof, 1> Robot::getPosition()
{
    return currentPosition;
}

void Robot::setHandlerFunctions(const std::function<void(double, Robot*)>& newSendCommandHandlerFunction,
        const std::function<void(double, Robot*)>& newReadResultHandlerFunction)
{
    const std::lock_guard<std::mutex> lock(handlerFunctionMutex);

    sendCommandHandlerFunction = newSendCommandHandlerFunction;
    readResultHandlerFunction = newReadResultHandlerFunction;
}

void Robot::removeHandlerFunctions()
{
    setHandlerFunctions([](double cycleTime, Robot* robot){}, [](double cycleTime, Robot* robot){});
}

void Robot::shutdown()
{
    if (!shuttingDown)
    {
        shuttingDown = true;
        t.join();
    }
}

double Robot::getCycleSleepTime()
{
    return cycleSleepTime;
}

Robot::Reference::Reference()
{
    for (int i = 0; i != dof; ++i)
    {
        trajItem.p[i] = 0;
        trajItem.v[i] = 0;
        trajItem.u[i] = 0;
    }
    this->gripperPos = 0;
}

Robot::Reference::Reference(Eigen::Matrix<double, dof, 1> pos, double gripperPos)
{
    for (int i = 0; i != dof; ++i)
    {
        trajItem.p[i] = pos[i];
        trajItem.v[i] = 0;
        trajItem.u[i] = 0;
    }
    this->gripperPos = gripperPos;
}

Robot::Reference::Reference(TrajectoryItem<dof, double> trajItem, double gripperPos)
{
    for (int i = 0; i != dof; ++i)
    {
        this->trajItem.p[i] = trajItem.p[i];
        this->trajItem.v[i] = trajItem.v[i];
        this->trajItem.u[i] = trajItem.u[i];
    }
    this->gripperPos = gripperPos;
}

Robot::Reference operator*(const Robot::Reference& a, float t)
{
    Robot::Reference out{a};
    out.trajItem = out.trajItem * t;
    out.gripperPos *= t;
    return out;
}

Robot::Reference operator*(float t, const Robot::Reference& a)
{
    return a * t;
}

Robot::Reference operator+(const Robot::Reference& a, const Robot::Reference& b)
{
    Robot::Reference out{a};
    out.trajItem = out.trajItem + b.trajItem;
    out.gripperPos += b.gripperPos;
    return out;
}

Robot::Reference operator-(const Robot::Reference& a, const Robot::Reference& b)
{
    Robot::Reference out{a};
    out.trajItem = out.trajItem - b.trajItem;
    out.gripperPos -= b.gripperPos;
    return out;
}

Robot::Reference operator/(const Robot::Reference& a, float t)
{
    Robot::Reference out{a};
    out.trajItem = out.trajItem / t;
    out.gripperPos /= t;
    return out;
}
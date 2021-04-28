#include "MasterCommunication.h"
#include "DCServoCommunicator.h"

#include "DummyTrajectoryGenerator.h"
#include "PathAndMoveBuilder.h"

#include <numeric>
#include <cmath>
#define pi (M_PI)

#include <chrono>
#include <thread>
#include <algorithm>
#include <functional>
#include <mutex>

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

        dcServoArray[0].setControlSpeed(10);
        dcServoArray[0].setBacklashControlSpeed(10, 3.0, 0.00);
        dcServoArray[0].setFrictionCompensation(0);
        dcServoArray[1].setControlSpeed(20);
        dcServoArray[1].setBacklashControlSpeed(10, 3.0, 0.00);
        dcServoArray[1].setFrictionCompensation(0);
        dcServoArray[2].setControlSpeed(10);
        dcServoArray[2].setBacklashControlSpeed(10, 3.0, 0.00);
        dcServoArray[2].setFrictionCompensation(0);
        dcServoArray[3].setControlSpeed(20);
        dcServoArray[3].setBacklashControlSpeed(10, 3.0, 0.0);
        dcServoArray[3].setFrictionCompensation(0);
        dcServoArray[4].setControlSpeed(20);
        dcServoArray[4].setBacklashControlSpeed(10, 3.0, 0.0);
        dcServoArray[4].setFrictionCompensation(0);
        dcServoArray[5].setControlSpeed(20);
        dcServoArray[5].setBacklashControlSpeed(10, 3.0, 0.0);
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

void playPath(Robot& robot,
        PathAndMoveBuilder& pathBuilder,
        const double playbackSpeed = 1.0,
        const std::array<bool, 6> activeMove = {true, true, true, true, true, true}, std::ostream& outStream = std::cout)
{
    if (playbackSpeed > 1.0)
    {
        throw -1;
    }

    double dt{0.001};

    RobotParameters::DynamicRobotDynamics dynamics{dt};

    DummyTrajectoryGenerator trajGen{&dynamics, 0.012 * 2};

    auto startPos = robot.getPosition();
    trajGen.setStart(startPos, 0.1);
    pathBuilder.renderTo(trajGen, startPos);

    trajGen.calculateTrajectory();

    bool doneRunning = false;
    bool reachedEndOfTrajectory = false;

    auto iter = std::cbegin(trajGen);
    auto endIter = std::cend(trajGen);
    auto outK = *iter;
    ++iter;
    auto outKp1 = *iter;
    auto pwm = outK.u;
    double playbackSpeedT = 0;
    SamplingHandler<TrajectoryItem<6, double> > sampler([&]()
            {
                auto outJ = interpolate(outK, outKp1, playbackSpeedT);
                playbackSpeedT += playbackSpeed;
                if (playbackSpeedT >= 1.0)
                {
                    playbackSpeedT -= 1.0;
                    outK = outKp1;
                    if (iter == endIter)
                    {
                        reachedEndOfTrajectory = true;
                    }
                    ++iter;
                    outKp1 = *iter;
                }
                auto outJp1 = interpolate(outK, outKp1, playbackSpeedT);
                outJ.v *= playbackSpeed;
                outJp1.v *= playbackSpeed;
                dynamics.recalculateFreedForward(outJ, outJp1);

                outJ.u[3] = 0;
                outJ.u[4] = 0;
                outJ.u[5] = 0;

                pwm = dynamics.recalcPwm(outJ.u, outJ.v);
                return outJ;
            }, dt);

    auto sendCommandHandlerFunction = [&sampler, &activeMove](double dt, Robot* robot)
        {
            auto& servos = robot->dcServoArray;

            auto trajItem = sampler.getSample();
            sampler.increment(dt);

            for (size_t i = 0; i != servos.size(); ++i)
            {
                if (activeMove[i])
                {
                    servos[i].setReference(trajItem.p[i], trajItem.v[i], trajItem.u[i]);
                }
            }
        };

    double t = 0;
    EigenVectord6 lastTempC;
    auto readResultHandlerFunction = [&t, &lastTempC, &doneRunning, &reachedEndOfTrajectory, &pwm, &outStream](double dt, Robot* robot)
        {
            auto& servos = robot->dcServoArray;

            bool communicationError = std::any_of(std::begin(servos), std::end(servos), [](auto& d)
                    {
                        return !d.isCommunicationOk();
                    });

            EigenVectord6 posJ;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(posJ),
                    [](const auto& c){return c.getPosition();});

            EigenVectord6 velJ;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(velJ),
                    [](const auto& c){return c.getBacklashCompensation();});

            EigenVectord6 erroJ;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(erroJ),
                    [](const auto& c){return c.getControlError(true);});

            EigenVectord6 controlSignal;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(controlSignal),
                    [](const auto& c){return c.getControlError(false);});

            JointSpaceCoordinate tempJ{posJ};
            CartesianCoordinate tempC{tempJ};

            if (t == 0)
            {
                lastTempC = tempC.c;
            }
            EigenVectord6 tempCVel = (tempC.c - lastTempC) / dt;
            lastTempC = tempC.c;

            const EigenVectord6& viewPos = tempC.c;
            const EigenVectord6& viewVel = tempCVel;

            std::string printVecName;
            auto printVecFunc = [&printVecName, &outStream](int i, const auto& v)
                    {
                        outStream << " " << printVecName << i << ":" << v;
                        return ++i;
                    };

            outStream << "t:" << t;
            printVecName = "p";
            std::accumulate(std::cbegin(viewPos), std::cend(viewPos), 0, printVecFunc);
            printVecName = "pj";
            std::accumulate(std::cbegin(posJ), std::cend(posJ), 0, printVecFunc);
            printVecName = "v";
            std::accumulate(std::cbegin(viewVel), std::cend(viewVel), 0, printVecFunc);
            printVecName = "vj";
            std::accumulate(std::cbegin(velJ), std::cend(velJ), 0, printVecFunc);
            printVecName = "e";
            std::accumulate(std::cbegin(erroJ), std::cend(erroJ), 0, printVecFunc);
            printVecName = "u";
            std::accumulate(std::cbegin(controlSignal), std::cend(controlSignal), 0, printVecFunc);
            outStream << "\n";

            t += dt;

            if (reachedEndOfTrajectory || communicationError)
            {
                robot->removeHandlerFunctions();
                doneRunning = true;
            }
        };

    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

    while(!doneRunning)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void moveGripper(Robot& robot, double pos, double time = 1.0)
{
    pos = asin(2.0 * pos - 1.0);

    bool doneRunning = false;

    double t = 0;
    double startPos = robot.gripperServo.getPosition();
    double amp = pos - startPos;
    auto sendCommandHandlerFunction = [&](double dt, Robot* robotPointer)
        {
            t = std::min(t + dt / time, 1.0);

            double pRef = startPos + amp * (1.0 - cos(pi * t)) / 2.0;
            robotPointer->gripperServo.setReference(pRef, 0.0, 0.0);
        };

    auto readResultHandlerFunction = [&](double dt, Robot* robotPointer)
        {
            if (t >= 1.0)
            {
                robotPointer->removeHandlerFunctions();
                doneRunning = true;
            }
        };

    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

    while(!doneRunning)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void recordeOpticalEncoderData(Robot& robot, size_t i, float pwm, float time, std::ostream& outStream = std::cout)
{
    bool doneRunning = false;

    double t = 0;

    auto sendCommandHandlerFunction = [&i, &pwm, &t](double dt, Robot* robot)
        {
            auto& servos = robot->dcServoArray;

            if (pwm != 0.0)
            {
                servos[i].setOpenLoopControlSignal(std::min(pwm, static_cast<float>(t * pwm)), true);
            }
        };

    auto readResultHandlerFunction = [&t, &doneRunning, &i, time, &outStream](double dt, Robot* robot)
        {
            t += dt;
            auto& servos = robot->dcServoArray;
            auto opticalEncoderData = servos[i].getOpticalEncoderChannelData();
            outStream << opticalEncoderData.a << ", " 
                      << opticalEncoderData.b << ", "
                      << opticalEncoderData.minCostIndex << ", "
                      << opticalEncoderData.minCost << ", "
                      << servos[i].getVelocity() / servos[i].getScaling() << "\n";

            if (true)
            {
                std::cout << static_cast<int>(100 * t / time) << "%\r";
            }

            if (t >= time)
            {
                robot->removeHandlerFunctions();
                doneRunning = true;
            }
        };

    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

    while(!doneRunning)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void recordeMomentOfInertia(Robot& robot, size_t i, float amp, float freq, std::ostream& outStream = std::cout)
{
    std::for_each(std::begin(robot.dcServoArray), std::end(robot.dcServoArray), [](auto& d)
        {
            d.disableBacklashControl();
        });

    bool doneRunning = false;

    double t = 0;
    float pos = 0;
    float vel = 0;
    float acc = 0;

    auto startPos = robot.getPosition();

    auto sendCommandHandlerFunction = [&](double dt, Robot* robotPointer)
        {
            t += dt;

            const double freqScaling = freq * (2.0 * pi);
            pos = startPos[i] + amp * (1 - cos(t * freqScaling));
            vel = amp * sin(t * freqScaling) * freqScaling;
            acc = amp * cos(t * freqScaling) * freqScaling * freqScaling;

            auto& servos = robotPointer->dcServoArray;

            for (size_t j = 0; j != servos.size(); ++j)
            {
                if (i == j)
                {
                    servos[j].setReference(pos, vel, 0);
                }
                else
                {
                    servos[j].setReference(startPos[j], 0, 0);
                }
            }
        };

    const double runTime = std::ceil(15 * freq) / freq;

    auto readResultHandlerFunction = [&](double dt, Robot* robotPointer)
        {
            auto& servo = robotPointer->dcServoArray[i];

            outStream << "t: " << t << ", "
                        << "p: " << servo.getPosition() << ", "
                        << "v: " << servo.getVelocity() << ", "
                        << "u: " << servo.getControlSignal() << ", "
                        << "acc: " << acc << "\n";

            if (t >= runTime)
            {
                robotPointer->removeHandlerFunctions();
                doneRunning = true;
            }
        };

    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

    while(!doneRunning)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void recordeSystemIdentData(Robot& robot, size_t i, double pwmAmp, double frictionPwm,
        std::ostream& outStream = std::cout)
{
    std::for_each(std::begin(robot.dcServoArray), std::end(robot.dcServoArray), [](auto& d)
        {
            d.disableBacklashControl();
        });

    bool doneRunning = false;

    auto pwmTestVec = std::vector<double>();

    int nr = 10;
    for (int i = 4; i != nr + 1; ++i)
    {
        pwmTestVec.push_back(i * pwmAmp / nr + frictionPwm);
        pwmTestVec.push_back(frictionPwm);
        pwmTestVec.push_back(-(i * pwmAmp / nr + frictionPwm));
        pwmTestVec.push_back(-frictionPwm);
    }

    double t = 0;
    double pwm = 0;
    auto sendCommandHandlerFunction = [&](double dt, Robot* robotPointer)
        {
            auto& servo = robotPointer->dcServoArray[i];

            if (t < pwmTestVec.size())
            {
                pwm = pwmTestVec.at(static_cast<size_t>(t));
            }
            servo.setOpenLoopControlSignal(pwm, true);
        };

    double runTime = pwmTestVec.size();
    auto readResultHandlerFunction = [&](double dt, Robot* robotPointer)
        {
            t += dt;
            auto& servo = robotPointer->dcServoArray[i];

            outStream << "t: " << t << ", "
                        << "p: " << servo.getPosition(false) / servo.getScaling() << ", "
                        << "c: " << servo.getCurrent() << ", "
                        << "pwm: " << pwm << "\n";

            if (t >= runTime)
            {
                robotPointer->removeHandlerFunctions();
                doneRunning = true;
            }
        };

    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);

    while(!doneRunning)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

PathAndMoveBuilder createPath()
{
    PathAndMoveBuilder pathBuilder;

    VelocityLimiter jointVelocityLimiter(0.1, EigenVectord6{1, 1, 1, 1, 1, 1}, 3.0 / 0.5);
    VelocityLimiter cartesianVelocityLimiter(0.025, EigenVectord6{1, 1, 1, 0, 0, 0}, 0.2 / 0.02);
    cartesianVelocityLimiter.add(0.1, EigenVectord6{1, 1, 1, 0, 0, 0});
    cartesianVelocityLimiter.add(0.01, EigenVectord6{0, 0, 0, 1, 1, 1}, 1.0 / 0.1);
    cartesianVelocityLimiter.add(0.4, EigenVectord6{0, 0, 0, 1, 1, 1});
    std::shared_ptr<JointSpaceDeviationLimiter> noDeviationLimiterJoint = 
            std::make_shared<JointSpaceDeviationLimiter>(std::numeric_limits<double>::max());
    std::shared_ptr<JointSpaceDeviationLimiter> deviationLimiterJoint =
            std::make_shared<JointSpaceDeviationLimiter>(0.0001);
    std::shared_ptr<CartesianSpaceDeviationLimiter> deviationLimiterCartesian =
            std::make_shared<CartesianSpaceDeviationLimiter>(0.0001);
    deviationLimiterCartesian->add(0.0001, EigenVectord6{0.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    EigenVectord6 xV{0.1, 0.0, 0.0, 0, 0, 0};
    EigenVectord6 yV{0.0, 0.08, 0.0, 0, 0, 0};
    EigenVectord6 zV{0.0, 0.0, 0.08, 0, 0, 0};
    EigenVectord6 raV{0, 0, 0, 0.9, 0.0, 0.0};
    EigenVectord6 rbV{0, 0, 0, 0.0, 0.8, 0.0};

    JointSpaceCoordinate jointSpaceHome{{0.0, pi / 2, pi / 2, 0, 0, 0}};
    CartesianCoordinate tempCartCoord(jointSpaceHome);

    EigenVectord6 crossCenter{0.0, 0.1, -0.1, 0, 0, 0};
    crossCenter += tempCartCoord.c;

    CartesianCoordinate tempCartCoord2(crossCenter);
    JointSpaceCoordinate crossCenterJointSpace{tempCartCoord2};

    pathBuilder.append(JointSpaceLinearPath::create(crossCenterJointSpace,
        jointVelocityLimiter, jointVelocityLimiter, deviationLimiterJoint));

    if (true)
    {
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + xV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + xV + 0.05 * zV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - xV + 0.05 * zV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - xV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));

        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + yV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + yV + 0.05 * zV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - yV + 0.05 * zV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - yV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));

        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + zV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + zV + 0.05 * yV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - zV + 0.05 * yV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - zV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));

        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + raV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - raV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));

        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter + rbV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter - rbV},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));
        pathBuilder.append(CartesianSpaceLinearPath::create(
                CartesianCoordinate{crossCenter},
                cartesianVelocityLimiter, cartesianVelocityLimiter, deviationLimiterCartesian));

        pathBuilder.append(JointSpaceLinearPath::create(jointSpaceHome,
            jointVelocityLimiter, jointVelocityLimiter, deviationLimiterJoint));
    }

    pathBuilder.append(JointSpaceLinearPath::create({-0.5, 0.9, 0.9, 1.4, 0, 0},
            jointVelocityLimiter, VelocityLimiter{0.1}, deviationLimiterJoint));

    pathBuilder.append(JointSpaceLinearPath::create({0.5, pi / 2 + 0, pi / 2 + 0, 0, 1.4, 0},
            jointVelocityLimiter, VelocityLimiter{0.1}, deviationLimiterJoint));

    pathBuilder.append(JointSpaceLinearPath::create({-0.5, 0.9, 0.9, 0, 0, 1.4},
            jointVelocityLimiter, VelocityLimiter{0.1}, deviationLimiterJoint));

    pathBuilder.append(JointSpaceLinearPath::create({0, pi / 2 + 0, pi / 2 + 0, 0, 0, 0},
            jointVelocityLimiter, VelocityLimiter{0.1}, deviationLimiterJoint));

    return pathBuilder;
}

#include <fstream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    // Declare the supported options.
    po::options_description options("Allowed options");
    options.add_options()
        ("servoNr", po::value<int>(), "servo nr")
        ("amp", po::value<double>(), "amplitude for recMomentOfInertia")
        ("frq", po::value<double>(), "frequency for recMomentOfInertia")
        ("pwmAmp", po::value<int>(), "pwm amplitude for recOpticalEncoder and recSystemIdentData")
        ("fricPwmAmp", po::value<int>(), "pwm amplitude to overcome friction")
        ("recOpticalEncoder", "record optical encoder data of given servo")
        ("recSystemIdentData", "record system ident data of given servo")
        ("recMomentOfInertia", "record moment of inertia data of given servo")
        ("playPath", "play the path defined in createPath()")
        ("output", po::value<std::string>(), "data output file")
        ("simulate", "simulate servos");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, options), vm);
    po::notify(vm);

    int numberOfArgs = std::accumulate(std::cbegin(vm), std::cend(vm), 0,
            [](int i, const auto& v){return ++i;});
    if (numberOfArgs == 0)
    {
        std::cout << options;
        return 0;
    }

    int servoNr = -1;
    if (vm.count("servoNr"))
    {
        servoNr = vm["servoNr"].as<int>();
    }

    double amp = -1.0;
    if (vm.count("amp"))
    {
        amp = vm["amp"].as<double>();
    }

    double frq = -1.0;
    if (vm.count("frq"))
    {
        frq = vm["frq"].as<double>();
    }

    int pwmAmp = -1;
    if (vm.count("pwmAmp"))
    {
        pwmAmp = vm["pwmAmp"].as<int>();
    }

    int fricPwmAmp = -1;
    if (vm.count("fricPwmAmp"))
    {
        fricPwmAmp = vm["fricPwmAmp"].as<int>();
    }

    std::unique_ptr<std::ofstream> outFileStream{nullptr};
    std::ostream* outStream = &std::cout;
    if (vm.count("output"))
    {
        std::string fileName = vm["output"].as<std::string>();
        outFileStream = std::make_unique<std::ofstream>(fileName);
        outStream = outFileStream.get();
    }

    std::unique_ptr<Communication> communication{nullptr};
    if (vm.count("simulate"))
    {
        communication = std::make_unique<SimulateCommunication>();
    }
    else
    {
        try
        {
            communication = std::make_unique<SerialCommunication>("/dev/ttyACM0");
        }
        catch (std::exception& e)
        {
            std::cout << "could not connect to robot serial port\n"; 
            std::cout << e.what();

            return 0;
        }
    }

    double comCycleTime = 0.018;
    std::array<bool, 7> servoSimulated{{false, false, false, false, false, false, true}};
    if (servoNr != -1)
    {
        if (vm.count("recSystemIdentData") == false)
        {
            comCycleTime = 0.004;
        }
        servoSimulated.fill(true);
        servoSimulated[servoNr - 1] = false;
    }
    Robot robot(communication.get(), servoSimulated, comCycleTime);

    if (vm.count("recOpticalEncoder"))
    {
        if (servoNr != -1)
        {
            if (pwmAmp == -1)
            {
                std::cout << "pwmAmp = ?\n";
                std::cin >> pwmAmp;
            }
            recordeOpticalEncoderData(robot, std::max(servoNr - 1, 0), pwmAmp, 200, *outStream);
        }
        else
        {
            std::cout << "no servo selected\n";
        }
    }
    else if (vm.count("recSystemIdentData"))
    {
        if (servoNr != -1)
        {
            if (pwmAmp == -1)
            {
                std::cout << "pwmAmp = ?\n";
                std::cin >> pwmAmp;
            }
            if (fricPwmAmp == -1)
            {
                std::cout << "fricPwmAmp = ?\n";
                std::cin >> fricPwmAmp;
            }
            recordeSystemIdentData(robot, std::max(servoNr - 1, 0), pwmAmp, fricPwmAmp, *outStream);
        }
        else
        {
            std::cout << "no servo selected\n";
        }
    }
    else if (vm.count("recMomentOfInertia"))
    {
        if (servoNr != -1)
        {
            if (amp == -1.0)
            {
                std::cout << "amp = ?\n";
                std::cin >> amp;
            }
            if (frq < 0.0)
            {
                std::cout << "frq = ?\n";
                std::cin >> frq;
            }
            recordeMomentOfInertia(robot, std::max(servoNr - 1, 0), amp, frq, *outStream);
        }
        else
        {
            std::cout << "no servo selected\n";
        }
    }
    else if (vm.count("playPath"))
    {
        try
        {
            PathAndMoveBuilder pathBuilder{createPath()};
            playPath(robot, pathBuilder, 1.0, {true, true, true, true, true, true}, *outStream);
        }
        catch (std::exception& e)
        {
            std::cout << e.what(); 
        }
    }
    else
    {
        std::cout << options;
    }

    robot.shutdown();

    return 0;
}

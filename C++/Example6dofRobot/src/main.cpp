#include "DummyTrajectoryGenerator.h"
#include "PathAndMoveBuilder.h"

#include "Robot.h"

#include "gui.h"

#include <numeric>
#include <cmath>

void playPath(Robot& robot,
        const std::vector<Robot::Reference> trajectory,
        const double playbackSpeed = 1.0,
        const std::array<bool, 7> activeMove = {true, true, true, true, true, true, true}, std::ostream& outStream = std::cout)
{
    if (playbackSpeed > 1.0)
    {
        throw -1;
    }

    double dt = 0.001;

    RobotParameters::DynamicRobotDynamics dynamics{dt};

    bool doneRunning = false;
    bool reachedEndOfTrajectory = false;

    auto iter = std::cbegin(trajectory);
    auto endIter = std::cend(trajectory);
    auto outK = iter->trajItem;
    ++iter;
    auto outKp1 = iter->trajItem;
    auto pwm = outK.u;
    double playbackSpeedT = 0;
    SamplingHandler<Robot::Reference > sampler([&]()
            {
                auto outJ = interpolate(outK, outKp1, playbackSpeedT);
                double gripperPosKp1 = iter->gripperPos;
                playbackSpeedT += playbackSpeed;
                if (playbackSpeedT >= 1.0)
                {
                    playbackSpeedT -= 1.0;
                    ++iter;
                    if (iter == endIter)
                    {
                        reachedEndOfTrajectory = true;
                    }
                    else
                    {
                        outK = outKp1;
                        outKp1 = iter->trajItem;
                        gripperPosKp1 = iter->gripperPos;
                    }
                }
                auto outJp1 = interpolate(outK, outKp1, playbackSpeedT);
                outJ.v *= playbackSpeed;
                outJp1.v *= playbackSpeed;
                dynamics.recalculateFreedForward(outJ, outJp1);

                outJ.u = EigenVectord6::Zero();

                pwm = dynamics.recalcPwm(outJ.u, outJ.v);
                return Robot::Reference(outJ, gripperPosKp1);
            }, dt);

    auto sendCommandHandlerFunction = [&sampler, &activeMove](double dt, Robot* robot)
        {
            auto& servos = robot->dcServoArray;

            auto refObj = sampler.getSample();
            sampler.increment(dt);

            for (size_t i = 0; i != servos.size(); ++i)
            {
                if (activeMove[i])
                {
                    servos[i]->setReference(refObj.trajItem.p[i], refObj.trajItem.v[i], refObj.trajItem.u[i]);
                }
            }

            if (activeMove[6])
            {
                double pos = asin(1.998 * refObj.gripperPos - 0.999);
                robot->gripperServo->setReference(pos, 0.0, 0.0);
            }
        };

    double t = 0;
    EigenVectord6 lastTempC;
    auto readResultHandlerFunction = [&t, &lastTempC, &doneRunning, &reachedEndOfTrajectory, &pwm, &outStream](double dt, Robot* robot)
        {
            auto& servos = robot->dcServoArray;

            bool communicationError = std::any_of(std::begin(servos), std::end(servos), [](auto& d)
                    {
                        return !d->isCommunicationOk();
                    });

            EigenVectord6 posJ;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(posJ),
                    [](const auto& c){return c->getPosition();});

            EigenVectord6 velJ;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(velJ),
                    [](const auto& c){return c->getBacklashCompensation();});

            EigenVectord6 erroJ;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(erroJ),
                    [](const auto& c){return c->getControlError(true);});

            EigenVectord6 controlSignal;
            std::transform(std::cbegin(servos), std::cend(servos), std::begin(controlSignal),
                    [](const auto& c){return c->getControlError(false);});

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

std::vector<Robot::Reference> moveGripper(double pos, double time, Robot::Reference& startRef)
{
    double dt = 0.001;
    double t = 0;
    double startPos = startRef.gripperPos;
    double amp = pos - startPos;

    std::vector<Robot::Reference> out;

    while (t < 1.0)
    {
        t = std::min(t + dt / time, 1.0);

        double pRef = startPos + amp * (1.0 - cos(pi * t)) / 2.0;
        out.push_back(Robot::Reference(startRef.trajItem, pRef));
    };

    return out;
}

std::vector<Robot::Reference> wait(double time, Robot::Reference& startRef)
{
    double dt = 0.001;
    double t = 0;

    std::vector<Robot::Reference> out;

    while (t < time)
    {
        t = t + dt;

        out.push_back(startRef);
    };

    return out;
}

std::vector<Robot::Reference> renderPath(
        DummyTrajectoryGenerator& trajGen,
        PathAndMoveBuilder& builder,
        Robot::Reference& startRef)
{
    const EigenVectord6& startPos = startRef.trajItem.p;

    trajGen.clear();
    trajGen.setStart(startPos, 0.1);
    builder.renderTo(trajGen, startPos);

    trajGen.calculateTrajectory();

    std::vector<Robot::Reference> out;

    auto endIt = std::cend(trajGen);
    auto it = std::cbegin(trajGen);
    ++it;
    for (; it != endIt; ++it)
    {
        out.push_back(Robot::Reference(*it, startRef.gripperPos));
    }

    return out;
}

void concatenate(std::vector<Robot::Reference>& a, const std::vector<Robot::Reference>& b)
{
    a.insert(std::end(a), std::cbegin(b), std::cend(b));
}

std::vector<Robot::Reference> rescaleToTimeMultiple(const std::vector<Robot::Reference>& a, double timeSlot)
{
    std::vector<Robot::Reference> out;
    double dt = 0.001;

    double trajLength = a.size() * dt;
    double playbackSpeed = trajLength / (std::ceil(trajLength / timeSlot) * timeSlot);

    auto it = std::cbegin(a);
    auto endIt = std::cend(a);

    double t = 0;
    auto outK = *it;
    ++it;
    auto outKp1 = *it;
    while (true)
    {
        auto inter = interpolate(outK, outKp1, t);
        inter.trajItem.v *= playbackSpeed;
        inter.trajItem.u *= 0;
        out.push_back(inter);

        t += playbackSpeed;
        if (t >= 1.0)
        {
            t -= 1.0;
            ++it;
            if (it == endIt)
            {
                break;
            }
            outK = outKp1;
            outKp1 = *it;
        }
    }

    return out;
}

std::vector<Robot::Reference> createTrajectory(Robot::Reference& startRef)
{
    double dt{0.001};

    RobotParameters::DynamicRobotDynamics dynamics{dt};

    DummyTrajectoryGenerator trajGen{&dynamics, 0.012 * 2};

    PathAndMoveBuilder pathBuilder;

    // velocities
    VelocityLimiter jVel(0.1, EigenVectord6{1, 1, 1, 1, 1, 1}, 3.0 / 0.10);
    jVel.add(3.0, EigenVectord6{1, 1, 1, 1, 1, 1});

    VelocityLimiter jLowVel(0.01, EigenVectord6{1, 1, 1, 1, 1, 1}, 0.3 / 0.25);
    jLowVel.add(0.3, EigenVectord6{1, 1, 1, 1, 1, 1});

    VelocityLimiter cVel(0.005, EigenVectord6{1, 1, 1, 0, 0, 0}, 0.2 / 0.01);
    cVel.add(0.2, EigenVectord6{1, 1, 1, 0, 0, 0});
    cVel.add(0.157, EigenVectord6{0, 0, 0, 1, 1, 1}, 15.7 / 1.0);
    cVel.add(15.7, EigenVectord6{0, 0, 0, 1, 1, 1});

    VelocityLimiter cLowVel(0.001, EigenVectord6{1, 1, 1, 0, 0, 0}, 0.06 / 0.02);
    cLowVel.add(0.06, EigenVectord6{1, 1, 1, 0, 0, 0});
    cLowVel.add(0.0157, EigenVectord6{0, 0, 0, 1, 1, 1}, 1.57 / 2.0);
    cVel.add(1.57, EigenVectord6{0, 0, 0, 1, 1, 1});

    VelocityLimiter zVel(0.0);

    std::shared_ptr<JointSpaceDeviationLimiter> divLimNo = 
            std::make_shared<JointSpaceDeviationLimiter>(std::numeric_limits<double>::max());
    std::shared_ptr<JointSpaceDeviationLimiter> divLimJ =
            std::make_shared<JointSpaceDeviationLimiter>(0.0001);
    std::shared_ptr<CartesianSpaceDeviationLimiter> divLimC =
            std::make_shared<CartesianSpaceDeviationLimiter>(0.0001);
    divLimC->add(0.0001, EigenVectord6{0.0, 0.0, 0.0, 1.0, 1.0, 1.0});

    // positions
    auto core1Top = CartesianCoordinate{{0.055751, -0.168444, 0.0247518, -0.115982, -0.0510046, 0.0477579}};
    auto core2Top = CartesianCoordinate{{0.0285, -0.166948, 0.024749, -0.115985, -0.0328628, 0.043177}};
    auto core3Top = CartesianCoordinate{{0.00061186, -0.165291, 0.025019, -0.115985, -0.0100289, 0.04327963}};
    auto core4Top = CartesianCoordinate{{-0.0263315, -0.164802, 0.0265809, -0.115985, -0.0126924, 0.0466}};

    auto core1Buttom = CartesianCoordinate{{0.053802, -0.167856, -0.012256, -0.115982, -0.0510046, 0.0477579}};
    auto core2Buttom = CartesianCoordinate{{0.02667, -0.167064, -0.0118657, -0.115985, -0.0328628, 0.043177}};
    auto core3Buttom = CartesianCoordinate{{-0.000185868, -0.16577, -0.0118581, -0.115985, -0.0100289, 0.0432796}};
    auto core4Buttom = CartesianCoordinate{{-0.0269217, -0.164659, -0.0117051, -0.115985, -0.0126924, 0.0466}};

    auto core1Above = CartesianCoordinate{core1Top.c + EigenVectord6{0, 0, 0.03, 0, 0, 0}};

    auto core1Up = CartesianCoordinate{0.7 * core1Top.c + 0.3 * core1Buttom.c};
    auto core2Up = CartesianCoordinate{0.7 * core2Top.c + 0.3 * core2Buttom.c};
    auto core3Up = CartesianCoordinate{0.7 * core3Top.c + 0.3 * core3Buttom.c};

    auto core1UpSide = CartesianCoordinate{core1Up.c + EigenVectord6{-0.006, 0, 0.004, 0, 0, 0}};
    auto core2UpSide = CartesianCoordinate{core2Up.c + EigenVectord6{-0.006, 0, 0.004, 0, 0, 0}};
    auto core3UpSide = CartesianCoordinate{core3Up.c + EigenVectord6{-0.006, 0, 0.004, 0, 0, 0}};

    auto core2Angled = CartesianCoordinate{core2Top.c + EigenVectord6{0, 0, 0, 0, 0, 0.5}};
    auto core3Angled = CartesianCoordinate{core3Top.c + EigenVectord6{0, 0, 0, 0, 0, 0.5}};
    auto core4Angled = CartesianCoordinate{core4Top.c + EigenVectord6{0, 0, 0, 0, 0, 0.5}};

    auto core1Down = CartesianCoordinate{core1Buttom.c + EigenVectord6{-0.006, 0, -0.003, 0, 0, 0}};
    auto core2Down = CartesianCoordinate{core2Buttom.c + EigenVectord6{-0.006, 0, -0.003, 0, 0, 0}};
    auto core3Down = CartesianCoordinate{core3Buttom.c + EigenVectord6{-0.006, 0, -0.003, 0, 0, 0}};

    auto core1ToCore4Half = CartesianCoordinate{(core1Top.c + core4Top.c) / 2};
    auto core4Half = CartesianCoordinate{(core4Top.c + core4Buttom.c) / 2};
    auto core4HalfAngled = CartesianCoordinate{core4Half.c + EigenVectord6{0, 0, 0, 0, 0, -0.8}};

    auto what1Position = JointSpaceCoordinate{{-0.314849, 1.67089, 2.23175, -0.00517668, 0.392985, -0.0108841}};
    auto what2Position = JointSpaceCoordinate{{-0.311014, 1.67013, 2.2339, 0.0164429, 0.394759, 0.355737}};
    auto yes1Position = JointSpaceCoordinate{{-0.311014, 1.67013, 2.2339, 0.0164429, 0.294653, 0.355737}};
    auto yes2Position = what2Position;

    auto pencilAbove = CartesianCoordinate{{0.111542, 0.00123832, 0.0343832, 1.55741, -0.542409, -0.0473626}};
    auto pencilUp = CartesianCoordinate{{0.117947, 0.00189401, -0.00601399, 1.55416, -0.832172, -0.0170461}};
    auto pencilDown = CartesianCoordinate{{0.095577, 2.61736e-05, -0.0282569, 1.4828, -0.808071, 0.0287696}};

    auto robotStartEnd = JointSpaceCoordinate{{0.508515, 1.82928, 2.81677, -0.209915, 0.198936, 0.137097}};

#define SYNC(timeSlot) concatenate(out, rescaleToTimeMultiple(renderPath(trajGen, pathBuilder, out.back()), (timeSlot))); pathBuilder.clear();

    // path
    std::vector<Robot::Reference> out;
    out.push_back(startRef);

    concatenate(out, moveGripper(0.8, 2 * 60.0 / 100, out.back()));

    pathBuilder.append(JointSpaceLinearPath::create(
        robotStartEnd, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)

    pathBuilder.append(JointSpaceLinearPath::create(
        pencilAbove, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        pencilUp, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        pencilDown, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    concatenate(out, moveGripper(0.22, 2 * 60.0 / 100, out.back()));
    pathBuilder.append(CartesianSpaceLinearPath::create(
        pencilUp, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(JointSpaceLinearPath::create(
        pencilAbove, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)

    pathBuilder.append(JointSpaceLinearPath::create(
        core1Above, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Above, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Above, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Above, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2Angled, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3Angled, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Angled, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Angled, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3Buttom, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3Up, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3UpSide, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3Down, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core3Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1ToCore4Half, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2Buttom, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2Up, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2UpSide, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2Down, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core2Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Above, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Above, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Above, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Above, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Buttom, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Up, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1UpSide, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Down, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core1ToCore4Half, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Half, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Top, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(JointSpaceLinearPath::create(
        what1Position, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)
    pathBuilder.append(JointSpaceLinearPath::create(
        what2Position, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)

    concatenate(out, wait(3 * 60.0 / 100, out.back()));

    pathBuilder.append(JointSpaceLinearPath::create(
        yes1Position, jVel, jVel, divLimJ));
    pathBuilder.append(JointSpaceLinearPath::create(
        yes2Position, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)
    pathBuilder.append(JointSpaceLinearPath::create(
        yes1Position, jVel, jVel, divLimJ));
    pathBuilder.append(JointSpaceLinearPath::create(
        yes2Position, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)

    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Half, cLowVel, cLowVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4HalfAngled, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        core4Top, cVel, cVel, divLimC));
    SYNC(60.0 / 100)

    pathBuilder.append(JointSpaceLinearPath::create(
        pencilAbove, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        pencilUp, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(CartesianSpaceLinearPath::create(
        pencilDown, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    concatenate(out, moveGripper(0.8, 2 * 60.0 / 100, out.back()));
    pathBuilder.append(CartesianSpaceLinearPath::create(
        pencilUp, cVel, cVel, divLimC));
    SYNC(60.0 / 100)
    pathBuilder.append(JointSpaceLinearPath::create(
        pencilAbove, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)

    pathBuilder.append(JointSpaceLinearPath::create(
        robotStartEnd, jVel, jVel, divLimJ));
    SYNC(60.0 / 100)

    return out;
}

#include <fstream>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    // Declare the supported options.
    po::options_description options("Allowed options");
    options.add_options()
        ("playPath", "play the path defined in createTrajectory()")
        ("gui", "jogging gui")
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
    std::array<bool, 7> servoSimulated{{false, false, false, false, false, false, false}};
    Robot robot(communication.get(), servoSimulated, comCycleTime);

    if (vm.count("gui"))
    {
        auto app = Gtk::Application::create("org.gtkmm.example");

        RobotJogging win(robot);
        app->run(win);
    }
    else if (vm.count("playPath"))
    {
        try
        {
            Robot::Reference startRef(robot.getPosition(), 1.0);

            playPath(robot, createTrajectory(startRef), 1.0, {true, true, true, true, true, true, true}, *outStream);
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

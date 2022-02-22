#include "Robot.h"

double pi = M_PI;

Robot::Robot(Communication* communication, const std::array<bool, 7> simulate, double cycleTime) :
        servoManager(cycleTime, [this, communication, &simulate](){
                return this->initFunction(communication, simulate);
            })
{
    auto& servos = servoManager.servos;
    for (size_t i = 0; i != dcServoArray.size(); ++i)
    {
        dcServoArray[i] = servos[i].get();
    }
    gripperServo = servos[6].get();
}

Robot::~Robot()
{
}

std::vector<std::unique_ptr<DCServoCommunicator> > Robot::initFunction(
        Communication* communication, const std::array<bool, 7> simulate)
{
    std::vector<std::unique_ptr<DCServoCommunicator> > servos;
    servos.push_back(std::make_unique<DCServoCommunicator>(1, simulate[0] ? communicationSim.get() : communication));
    servos.push_back(std::make_unique<DCServoCommunicator>(2, simulate[1] ? communicationSim.get() : communication));
    servos.push_back(std::make_unique<DCServoCommunicator>(3, simulate[2] ? communicationSim.get() : communication));
    servos.push_back(std::make_unique<DCServoCommunicator>(4, simulate[3] ? communicationSim.get() : communication));
    servos.push_back(std::make_unique<DCServoCommunicator>(5, simulate[4] ? communicationSim.get() : communication));
    servos.push_back(std::make_unique<DCServoCommunicator>(6, simulate[5] ? communicationSim.get() : communication));
    servos.push_back(std::make_unique<DCServoCommunicator>(7, simulate[6] ? communicationSim.get() : communication));

    servos[0]->setOffsetAndScaling(2 * pi / 4096.0, 0.832185, 0);
    servos[1]->setOffsetAndScaling(2 * pi / 4096.0, -2.042107923, pi / 2);
    servos[2]->setOffsetAndScaling(2 * pi / 4096.0, 1.0339, pi / 2);
    servos[3]->setOffsetAndScaling(-2 * pi / 4096.0, -1.22728, 0.0);
    servos[4]->setOffsetAndScaling(2 * pi / 4096.0, 0.8 * 1.42914 + 0.2 * 1.47876, 0.0);
    servos[5]->setOffsetAndScaling(-2 * pi / 4096.0, -1.27661, 0.0);

    servos[0]->setControlSpeed(25);
    servos[0]->setBacklashControlSpeed(12, 3.0, 0.00);
    servos[0]->setFrictionCompensation(0);
    servos[1]->setControlSpeed(25);
    servos[1]->setBacklashControlSpeed(12, 3.0, 0.00);
    servos[1]->setFrictionCompensation(0);
    servos[2]->setControlSpeed(25);
    servos[2]->setBacklashControlSpeed(12, 3.0, 0.00);
    servos[2]->setFrictionCompensation(0);
    servos[3]->setControlSpeed(30);
    servos[3]->setBacklashControlSpeed(12, 3.0, 0.0);
    servos[3]->setFrictionCompensation(0);
    servos[4]->setControlSpeed(30);
    servos[4]->setBacklashControlSpeed(12, 3.0, 0.0);
    servos[4]->setFrictionCompensation(0);
    servos[5]->setControlSpeed(30);
    servos[5]->setBacklashControlSpeed(12, 3.0, 0.0);
    servos[5]->setFrictionCompensation(0);

    //gripper servo handling
    servos[6]->setOffsetAndScaling(pi / 1900.0, 0.0, 0.0);

    while (!servos[6]->isInitComplete())
    {
        servos[6]->run();
    }

    for (size_t i = 0; i != 2; ++i)
    {
        servos[6]->setReference(pi / 2.0, 0.0, 0.0);
        servos[6]->run();
        servos[6]->getPosition();
    }

    return servos;
}

Eigen::Matrix<double, Robot::dof, 1> Robot::getPosition() const
{
    auto pos = servoManager.getPosition();
    Eigen::Matrix<double, Robot::dof, 1> out;
    for (size_t i = 0; i != dof; ++i)
    {
        out[i] = pos[i];
    }

    return out;
}

void Robot::setHandlerFunctions(const std::function<void(double, Robot&)>& newSendCommandHandlerFunction,
        const std::function<void(double, Robot&)>& newReadResultHandlerFunction,
        const std::function<void(std::exception_ptr e)>& newErrorHandlerFunction)
{
    std::function<void(double, ServoManager& manager)> sendFunc = [this, newSendCommandHandlerFunction](double dt, ServoManager& manager){
        newSendCommandHandlerFunction(dt, *this);
    };
    std::function<void(double, ServoManager& manager)> readFunc = [this, newReadResultHandlerFunction](double dt, ServoManager& manager){
        newReadResultHandlerFunction(dt, *this);
    };

    servoManager.setHandlerFunctions(sendFunc, readFunc, newErrorHandlerFunction);
}

void Robot::removeHandlerFunctions()
{
    servoManager.removeHandlerFunctions();
}

void Robot::start()
{
    servoManager.start();
}

void Robot::shutdown()
{
    servoManager.shutdown();
}

void Robot::enableDelayedExceptions(bool enable)
{
    servoManager.enableDelayedExceptions(enable);
}

std::exception_ptr Robot::getUnhandledException()
{
    return servoManager.getUnhandledException();
}

bool Robot::isAlive(bool raiseException)
{
    return servoManager.isAlive(raiseException);
}

double Robot::getCycleSleepTime() const
{
    return servoManager.getCycleSleepTime();
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
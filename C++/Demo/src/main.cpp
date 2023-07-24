#include "ServoProject.h"
#include <iostream>
#include <iomanip>
#include <cmath>

void playTrajectory(ServoManager& manager,
        const std::vector<std::vector<double> >& trajectory);

void addLinearMove(std::vector<std::vector<double> >& trajectory,
        const std::vector<double>& endPos,
        double duration);

void addSmoothMove(std::vector<std::vector<double> >& trajectory,
        const std::vector<double>& endPos,
        double duration);

void addWait(std::vector<std::vector<double> >& trajectory, double duration);

double dt = 0.018;

int main()
{
    std::string port = "";// "/dev/ttyACM0";
    std::shared_ptr<Communication> simCom = std::make_unique<SimulateCommunication>();
    std::shared_ptr<Communication> com;
    if (port == "")
    {
        std::cout << "Simulation mode active\n";
        com = simCom;
    }
    else
    {
        com = std::make_shared<SerialCommunication>(port);
    }

    auto initFun = [&com, &simCom]() {

        std::vector<std::unique_ptr<DCServoCommunicator> > servos;

        auto newServo = std::make_unique<DCServoCommunicator>(1, com.get());
        newServo->setOffsetAndScaling(360.0 / 4096.0, -110.0, 0);
        newServo->setControlSpeed(32, 1.2);
        newServo->setBacklashControlSpeed(6, 180.0, 0.00);
        servos.push_back(std::move(newServo));

        newServo = std::make_unique<DCServoCommunicator>(2, com.get());
        newServo->setOffsetAndScaling(180.0 / 1900.0, 0.0, 0.0);
        servos.push_back(std::move(newServo));

        return servos;
    };

    ServoManager manager(dt, initFun);

    std::vector<std::vector<double> > trajectory;
    auto startPos = manager.getPosition();
    trajectory.push_back(startPos);

    addSmoothMove(trajectory, {{0.0, 0.0}}, 1.0);

    addWait(trajectory, 10.0);

    for (size_t i = 0; i != 3; ++i)
    {
        //addSmoothMove(trajectory, {{2.0, 2.0}}, 0.2);
        addSmoothMove(trajectory, {{-10.0, -10.0}}, 1.0);
        addSmoothMove(trajectory, {{0.0, 0.0}}, 3.0);
        addWait(trajectory, 3.0);
        //addSmoothMove(trajectory, {{-2.0, -2.0}}, 0.2);
        addSmoothMove(trajectory, {{10.0, 10.0}}, 1.0);
        addSmoothMove(trajectory, {{0.0, 0.0}}, 3.0);
        addWait(trajectory, 3.0);
    }

    addWait(trajectory, 7.0);

    playTrajectory(manager, trajectory);

    return 0;
}

void playTrajectory(ServoManager& manager,
        const std::vector<std::vector<double> >& trajectory)
{
    if (trajectory.size() == 0)
    {
        return;
    }

    bool doneRunning = false;
    size_t index = 0;

    size_t dof = trajectory.back().size();
    for (const auto& pos : trajectory)
    {
        if (dof != pos.size())
        {
            throw std::length_error("Inconsistent position dimension");
        }
    }

    auto sendCommandHandlerFunction = [&](double dt, ServoManager& manager){
        std::vector<double> pos = trajectory.back();
        std::vector<double> vel(pos.size(), 0.0);

        if (index == 0)
        {
            pos = trajectory[0];
        }
        else if (index < trajectory.size() - 1)
        {
            pos = trajectory[index];

            auto calcVel = [dt](const auto& p0, const auto& p2){
                std::vector<double> out;

                for (size_t i = 0; i != p0.size(); ++i)
                {
                    out.push_back((p2[i] - p0[i]) / 2.0 / dt);
                }

                return out;
            };

            vel = calcVel(trajectory[index - 1], trajectory[index + 1]);
        }

        size_t maxIndex = std::min(manager.servos.size(), pos.size());
        for (size_t i = 0; i != maxIndex; ++i)
        {
            manager.servos[i]->setReference(pos[i], vel[i], 0.0);
        }

        ++index;
        if (index == trajectory.size())
        {
            manager.removeHandlerFunctions();
            doneRunning = true;
        }
    };

    auto readResultHandlerFunction = [&](double dt, ServoManager& manager)
    {
        std::vector<double> pos = manager.getPosition();

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "pos[0] = " << std::setw(7) << pos[0]
                << ", pos[1] = " << std::setw(7) << pos[1] << "\r" << std::flush;
    };

    auto errorHandlerFunction = [&manager](std::exception_ptr e){
            try
            {
                 std::rethrow_exception(e);
            }
            catch (CommunicationError& comExc)
            {
                std::cout << comExc.what() << "\n";
                if (comExc.code != CommunicationError::COULD_NOT_SEND)
                {
                    std::cout << "exception triggered: restarting communication...\n";
                    manager.shutdown();
                    manager.start();
                }
                else
                {
                    throw;
                }
            }
        };

    manager.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction, errorHandlerFunction);

    while (!doneRunning)
    {
        if (!manager.isAlive())
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

std::vector<double> interpolateVectors(
        const std::vector<double>& pos0,
        const std::vector<double>& pos1, double t)
{
    if (pos0.size() != pos1.size())
    {
        throw std::length_error("Inconsistent position dimension");
    }

    std::vector<double> out;

    for (size_t i = 0; i != pos0.size(); ++i)
    {
        out.push_back(pos1[i] * t + pos0[i] * (1.0 - t));
    }
    return out;
}

void addLinearMove(std::vector<std::vector<double> >& trajectory,
        const std::vector<double>& endPos,
        double duration)
{
    auto startPos = trajectory.back();

    if (startPos.size() != endPos.size())
    {
        throw std::length_error("Inconsistent position dimension");
    }

    size_t steps = static_cast<int>(round(duration / dt));
    for (size_t i = 1; i != steps + 1; ++i)
    {
        double t = static_cast<double>(i) / steps;

        trajectory.push_back(interpolateVectors(startPos, endPos, t));
    }
}

void addSmoothMove(std::vector<std::vector<double> >& trajectory,
        const std::vector<double>& endPos,
        double duration)
{
    auto startPos = trajectory.back();

    if (startPos.size() != endPos.size())
    {
        throw std::length_error("Inconsistent position dimension");
    }

    size_t steps = static_cast<int>(round(duration / dt));
    for (size_t i = 1; i != steps + 1; ++i)
    {
        double t = static_cast<double>(i) / steps;
        t = (1.0 - cos(M_PI * t)) / 2.0;

        trajectory.push_back(interpolateVectors(startPos, endPos, t));
    }
}

void addWait(std::vector<std::vector<double> >& trajectory, double duration)
{
    auto startPos = trajectory.back();

    size_t steps = static_cast<int>(round(duration / dt));
    for (size_t i = 1; i != steps + 1; ++i)
    {
        trajectory.push_back(startPos);
    }
}

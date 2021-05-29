#include "gui.h"
#include "PathAndMoveBuilder.h"

#include <iostream>

RobotJogging::RobotJogging(Robot& r) :
    robot{r}
{
    set_border_width(10);
    set_default_size(800, 600);

    auto vBox = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    add(*vBox);

    button = Gtk::make_managed<Gtk::Button>("Enable");
    button->signal_clicked().connect(sigc::mem_fun(*this,
              &RobotJogging::onButtonClicked));
    vBox->add(*button);

    combo = Gtk::make_managed<Gtk::ComboBoxText>();
    combo->append("Joint space");
    combo->append("Cartesian ");
    combo->set_active(0);
    combo->signal_changed().connect(sigc::mem_fun(*this,
              &RobotJogging::onComboChanged));

    vBox->add(*combo);

    for (size_t i = 0; i != joggingScales.size(); ++i)
    {
        auto& scale = joggingScales[i];
        scale = Gtk::make_managed<JoggingScale>();
        vBox->add(*scale);
    }

    gripperScale = Gtk::make_managed<Gtk::Scale>(Gtk::Adjustment::create(1.0, 0.0, 1.0, 0.01, 0.1, 0.0), Gtk::ORIENTATION_HORIZONTAL);
    gripperScale->set_digits(2);
    gripperScale->signal_value_changed().connect(sigc::mem_fun(*this,
              &RobotJogging::onGripperChanged));
    vBox->add(*gripperScale);

    entry = Gtk::make_managed<Gtk::Entry>();
    vBox->add(*entry);

    vBox->show_all();

    sigc::slot<bool> my_slot = sigc::mem_fun(*this,
                  &RobotJogging::onTimeout);

    Glib::signal_timeout().connect(my_slot,
          100);

    auto sendCommandHandlerFunction = [&](double dt, Robot* robotPointer)
    {
        if (!controlEnabled)
        {
            enableTimer = 0;
        }
        else
        {
            enableTimer += dt;
        }

        Eigen::Matrix<double, 6, 1> velInJ;

        if (cartesian)
        {
            JointSpaceCoordinate currentInJ{currentPosition};
            CartesianCoordinate currentInC(currentInJ);
            CartesianCoordinate newPosInC(currentInC);

            for (int i = 0; i != 3; ++i)
            {
                double v = joggingScales[i]->getValue();

                newPosInC.c[i] += 0.1 * dt * v;
            }

            for (int i = 3; i != 6; ++i)
            {
                double v = joggingScales[i]->getValue();

                newPosInC.c[i] += dt * v;
            }

            JointSpaceCoordinate newPosInJ{newPosInC};

            velInJ = (newPosInJ.c - currentInJ.c) / dt;

            const double maxJointVel = 1.0;
            double t = 0;
            for (auto v : velInJ)
            {
                t = std::max(t, v / maxJointVel);
            }

            if (t > 1.0)
            {
                velInJ /= t;
            }

            robotThreadMutex.lock();
            currentPosition = currentInJ.c + velInJ * dt;
            robotThreadMutex.unlock();
        }
        else
        {
            robotThreadMutex.lock();
            for (int i = 0; i != currentPosition.size(); ++i)
            {
                velInJ[i] = joggingScales[i]->getValue();
                currentPosition[i] += dt * velInJ[i];
            }
            robotThreadMutex.unlock();
        }

        bool validCurrentPosition = std::all_of(std::cbegin(currentPosition), std::cend(currentPosition), [](double v)
            {
                return !std::isnan(v);
            });

        for (int i = 0; i != currentPosition.size(); ++i)
        {
            if (enableTimer > i * 0.1)
            {
                if (validCurrentPosition)
                {
                    robotPointer->dcServoArray[i].setReference(currentPosition[i], velInJ[i], 0.0);
                }
            }
            else
            {
                robotPointer->dcServoArray[i].setOpenLoopControlSignal(0, true);
            }
        }

        if (enableTimer > 6 * 0.1)
        {
            double pos = asin(2.0 * gripperPos - 1.0);
            robotPointer->gripperServo.setReference(pos, 0.0, 0.0);
        }
    };

    auto readResultHandlerFunction = [&](double dt, Robot* robotPointer)
    {
        robotThreadMutex.lock();
        for (int i = 0; i != currentPosition.size(); ++i)
        {
            if (enableTimer <= i * 0.1)
            {
                currentPosition[i] = robotPointer->dcServoArray[i].getPosition();
            }
        }
        robotThreadMutex.unlock();

        if (shutdown)
        {
            robotPointer->removeHandlerFunctions();
        }
    };

    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);
}

RobotJogging::~RobotJogging()
{
    shutdown = true;
    robot.shutdown();
}

void RobotJogging::onButtonClicked()
{
    controlEnabled = !controlEnabled;
    if (controlEnabled)
    {
        button->set_label("Disable");
    }
    else
    {
        button->set_label("Enable");
    }
}

void RobotJogging::onComboChanged()
{
    if (combo->get_active_row_number() == 1)
    {
        cartesian = true;
    }
    else
    {
        cartesian = false;
    }
}

void RobotJogging::onGripperChanged()
{
    double tempGripperPos = gripperScale->get_value();

    robotThreadMutex.lock();
    gripperPos = tempGripperPos;
    robotThreadMutex.unlock();
}

bool RobotJogging::onTimeout()
{
    int temp1;
    int temp2;
    if (entry->get_selection_bounds(temp1, temp2) && entry->has_focus())
    {
        entry->select_region(0, -1);
        return true;
    }

    robotThreadMutex.lock();
    auto currentPositionCopy = currentPosition;
    robotThreadMutex.unlock();

    std::stringstream ss("");
    if (cartesian)
    {
        JointSpaceCoordinate currentPositionCopyInJ(currentPositionCopy);
        CartesianCoordinate currentPositionCopyInC(currentPositionCopyInJ);
        std::string temp = "CartesianCoordinate{{";
        for (auto v : currentPositionCopyInC.c)
        {
            ss << temp;
            ss << v;
            temp = ", ";
        }
        ss << "}};";
    }
    else
    {
        std::string temp = "JointSpaceCoordinate{{";
        for (auto v : currentPositionCopy)
        {
            ss << temp;
            ss << v;
            temp = ", ";
        }
        ss << "}};";
    }

    entry->set_text(ss.str());

    return true;
}

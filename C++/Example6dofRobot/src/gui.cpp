#include "gui.h"
#include "PathAndMoveBuilder.h"

#include <iostream>

RobotJogging::RobotJogging(Robot& r) :
    robot{r}
{
    robot.enableDelayedExceptions();

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

    moveToPositionScale = Gtk::make_managed<JoggingScale>();
    vBox->add(*moveToPositionScale);

    moveToPositionEntry = Gtk::make_managed<Gtk::ComboBoxText>(true);
    moveToPositionEntry->append("");
    moveToPositionEntry->set_active(0);
    moveToPositionEntry->signal_changed().connect(sigc::mem_fun(*this,
              &RobotJogging::onMoveToPositionEntryChanged));
    vBox->add(*moveToPositionEntry);

    currentPosEntry = Gtk::make_managed<Gtk::Entry>();
    vBox->add(*currentPosEntry);

    vBox->show_all();

    sigc::slot<bool> my_slot = sigc::mem_fun(*this,
                  &RobotJogging::onTimeout);

    Glib::signal_timeout().connect(my_slot,
          100);

    auto sendCommandHandlerFunction = [&](double dt, Robot& robot)
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
            robotThreadMutex.lock();
            JointSpaceCoordinate currentInJ{currentPosition};
            CartesianCoordinate currentInC(currentInJ);
            CartesianCoordinate newPosInC(currentInC);

            JointSpaceCoordinate moveToPositionPosInJ{moveToPositionPos};
            CartesianCoordinate moveToPositionPosInC(moveToPositionPosInJ);

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

            double moveToPositionScaleValue =
                    moveToPositionScale->getValue();
            if (moveToPositionScaleValue != 0.0)
            {
                auto moveToPosVel = moveToPositionPosInC.c;
                moveToPosVel -= newPosInC.c;
                auto temp = moveToPosVel;
                temp[0] *= 10.0;
                temp[1] *= 10.0;
                temp[2] *= 10.0;
                moveToPosVel /= std::max(0.1, temp.norm());

                moveToPosVel *= moveToPositionScaleValue;

                newPosInC.c += moveToPosVel * dt;
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

            currentPosition = currentInJ.c + velInJ * dt;
            robotThreadMutex.unlock();
        }
        else
        {
            robotThreadMutex.lock();
            for (int i = 0; i != currentPosition.size(); ++i)
            {
                velInJ[i] = joggingScales[i]->getValue();
            }

            auto moveToPosVel = moveToPositionPos;
            moveToPosVel -= currentPosition;
            moveToPosVel /= std::max(0.1, moveToPosVel.norm());

            moveToPosVel *= moveToPositionScale->getValue();

            velInJ += moveToPosVel;
            currentPosition += velInJ * dt;
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
                    robot.dcServoArray[i]->setReference(currentPosition[i], velInJ[i], 0.0);
                }
            }
            else
            {
                robot.dcServoArray[i]->setOpenLoopControlSignal(0, true);
            }
        }

        if (enableTimer > 6 * 0.1)
        {
            double pos = asin(2.0 * gripperPos - 1.0);
            robot.gripperServo->setReference(pos, 0.0, 0.0);
        }
    };

    auto readResultHandlerFunction = [&](double dt, Robot& robot)
    {
        robotThreadMutex.lock();
        for (int i = 0; i != currentPosition.size(); ++i)
        {
            if (enableTimer <= i * 0.1)
            {
                currentPosition[i] = robot.dcServoArray[i]->getPosition();
            }
        }
        robotThreadMutex.unlock();

        if (shutdown)
        {
            robot.removeHandlerFunctions();
        }
    };

    robot.setHandlerFunctions(sendCommandHandlerFunction, readResultHandlerFunction);
}

RobotJogging::~RobotJogging()
{
    robot.enableDelayedExceptions(false);

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

void RobotJogging::onMoveToPositionEntryChanged()
{
    static int last = 0;
    static bool block = false;

    if (block)
    {
        return;
    }

    std::string text = moveToPositionEntry->get_active_text();

    if (moveToPositionEntry->get_active_row_number() == -1 &&
        last != -1)
    {
        block = true;
        std::cout << last << "\n";
        if (text[text.size() - 1] == '+')
        {
            last += 1;
            moveToPositionEntry->insert(last, " ");
            moveToPositionEntry->set_active(last);
        }
        else if (text.size() == 0 &&
                last != 0)
        {
            moveToPositionEntry->remove_text(last);
            last = -1;
            moveToPositionEntry->set_active(-1);
        }
        else
        {
            moveToPositionEntry->remove_text(last);
            moveToPositionEntry->insert(last, text);
        }
        block = false;
    }
    else
    {
        last = moveToPositionEntry->get_active_row_number();
    }

    setMoveToPositionPosFromString(text);
}

void RobotJogging::setMoveToPositionPosFromString(const std::string& str)
{
    bool cartesian = str.find("CartesianCoordinate") != std::string::npos;
    auto b = str.find("{{");
    auto e = str.find("}};");

    if (b != std::string::npos && e != std::string::npos)
    {
        b += 2;
        std::stringstream ss(str.substr(b, e - b));

        if (cartesian)
        {
            bool error = false;
            CartesianCoordinate c;
            for (auto& v : c.c)
            {
                ss >> v;
                error |= ss.fail();
                ss.ignore(std::numeric_limits<streamsize>::max(), ',');
            }

            error |= !ss.eof();

            if (error)
            {
                return;
            }

            JointSpaceCoordinate newPosInJ(c);
            robotThreadMutex.lock();
            moveToPositionPos = newPosInJ.c;
            robotThreadMutex.unlock();
        }
        else
        {
            bool error = false;
            JointSpaceCoordinate j;
            for (auto& v : j.c)
            {
                ss >> v;
                error |= ss.fail();
                ss.ignore(std::numeric_limits<streamsize>::max(), ',');
            }

            error |= !ss.eof();

            if (error)
            {
                return;
            }

            robotThreadMutex.lock();
            moveToPositionPos = j.c;
            robotThreadMutex.unlock();
        }
    }
}

bool RobotJogging::onTimeout()
{
    auto robotExc = robot.getUnhandledException();
    if (robotExc)
    {
        try
        {
             std::rethrow_exception(robotExc);
        }
        catch (CommunicationError& comExc)
        {
            std::cout << comExc.what() << "\n";
            if (comExc.code != CommunicationError::COULD_NOT_SEND)
            {
                std::cout << "exception triggered: restarting communication...\n";
                robot.shutdown();
                robot.start();
            }
            else
            {
                throw;
            }
        }
    }

    int temp1;
    int temp2;
    if (currentPosEntry->get_selection_bounds(temp1, temp2) && currentPosEntry->has_focus())
    {
        currentPosEntry->select_region(0, -1);
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

    currentPosEntry->set_text(ss.str());

    return true;
}

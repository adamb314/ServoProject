using namespace std;

#include <gtkmm.h>
#include <ctime>
#include <cmath>
#include <cairomm/context.h>
#include <glibmm/main.h>
#include <gtkmm/application.h>
#include <gtkmm/window.h>
#include <gtkmm/scale.h>
#include <gtkmm/button.h>
#include <gtkmm/comboboxtext.h>
#include <gtkmm/entry.h>
#include <gtkmm/object.h>
#include <thread>
#include <mutex>
#include <memory>

#include "Robot.h"

#ifndef GUI_IMP_H
#define GUI_IMP_H

class JoggingScale : public Gtk::Scale
{
public:
    JoggingScale() :
        Gtk::Scale(Gtk::Adjustment::create(0.0, -1.0, 1.0, 0.001, 0.01, 0.0), Gtk::ORIENTATION_HORIZONTAL)
    {
        set_digits(3);
        signal_button_press_event().connect(sigc::mem_fun(*this,
              &JoggingScale::onButtonPressed), false);
        signal_button_release_event().connect(sigc::mem_fun(*this,
              &JoggingScale::onButtonReleased), false);
        signal_value_changed().connect(sigc::mem_fun(*this,
              &JoggingScale::onValueChanged));

        sigc::slot<bool> my_slot = sigc::mem_fun(*this,
                      &JoggingScale::onTimeout);

        Glib::signal_timeout().connect(my_slot,
              20);
    }

    double getValue()
    {
        mutex.lock();
        double out = value;
        mutex.unlock();
        return out;
    }

private:
    bool onButtonPressed(GdkEventButton* event)
    {
        pressed = true;
        return false;
    }

    bool onButtonReleased(GdkEventButton* event)
    {
        pressed = false;
        return false;
    }

    bool onTimeout()
    {
        if (!pressed)
        {
            mutex.lock();
            if (value >= 0)
            {
                value -= 0.02 * (3.0 * std::abs(value) + 0.6);
                value = std::max(0.0, value);
            }
            else
            {
                value += 0.02 * (3.0 * std::abs(value) + 0.6);
                value = std::min(0.0, value);
            }
            mutex.unlock();
            set_value(value);
        }
        return true;
    }

    void onValueChanged()
    {
        mutex.lock();
        value = get_value();
        mutex.unlock();
    }

    bool pressed{false};
    double value;

    std::mutex mutex;
};

class RobotJogging : public Gtk::Window
{
public:
    RobotJogging(Robot& r);
    ~RobotJogging() override;

protected:
    void onButtonClicked();
    void onComboChanged();
    void onGripperChanged();
    void onMoveToPositionEntryChanged();
    bool onTimeout();

    void setMoveToPositionPosFromString(const std::string& str);

    Gtk::Button* button{nullptr};
    Gtk::ComboBoxText* combo;
    std::array<JoggingScale*, 6> joggingScales{nullptr};
    Gtk::Scale* gripperScale{nullptr};
    JoggingScale* moveToPositionScale{nullptr};
    Gtk::ComboBoxText* moveToPositionEntry{nullptr};
    Gtk::Entry* currentPosEntry{nullptr};
    bool controlEnabled{false};
    bool cartesian{false};
    bool shutdown{false};
    double enableTimer{0};
    Robot& robot;

    std::mutex robotThreadMutex;
    Eigen::Matrix<double, 6, 1> currentPosition;
    Eigen::Matrix<double, 6, 1> moveToPositionPos;
    double gripperPos{1.0};
};

#endif

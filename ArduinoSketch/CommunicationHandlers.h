#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "StatusLightHandler.h"
#include "Communication.h"
#include <Servo.h>

#ifndef COMMUNICATION_HANDLERS_H
#define COMMUNICATION_HANDLERS_H

class DCServoCommunicationHandler : public Communication<1>
{
public:
    DCServoCommunicationHandler(unsigned char nodeNr, unsigned long baud);

    ~DCServoCommunicationHandler();

    virtual void onReadyToSendEvent();

    virtual void onReceiveCompleteEvent();

    virtual void onErrorEvent();

    virtual void onComCycleEvent();

    void onComIdleEvent() override;

private:
    StatusLightHandler statusLight;
};

template <size_t N>
class ServoCommunicationHandler : public Communication<N>
{
public:
    ServoCommunicationHandler(std::array<unsigned char, N> servoPinArray,
            std::array<unsigned char, N> nodeNrArray,
            unsigned long baud) :
        Communication<N>(nodeNrArray, baud)
    {
        for (size_t i = 0; i != servoPinArray.size(); ++i)
        {
            servoArray[i].attach(servoPinArray[i], 0, 2000);
        }
    }

    ~ServoCommunicationHandler()
    {
    }

    virtual void onReadyToSendEvent()
    {
    }

    virtual void onReceiveCompleteEvent()
    {
    }

    virtual void onErrorEvent()
    {
    }

    virtual void onComCycleEvent()
    {

        for (size_t i = 0; i != servoArray.size(); ++i)
        {
            if (Communication<N>::intArrayChanged[i][0])
            {
                Communication<N>::intArrayChanged[i][0] = false;

                servoArray[i].writeMicroseconds(Communication<N>::intArray[i][0]);
            }
            else
            {
                servoArray[i].writeMicroseconds(0);
            }
        }

        statusLight.showDisabled();

        statusLight.showCommunicationActive();
    }

    void onComIdleEvent() override
    {
        for (size_t i = 0; i != servoArray.size(); ++i)
        {
            servoArray[i].writeMicroseconds(0);
        }

        statusLight.showDisabled();
        statusLight.showCommunicationInactive();
    }

private:
    std::array<Servo, N> servoArray;
    StatusLightHandler statusLight;
};

#endif

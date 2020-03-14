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

#endif

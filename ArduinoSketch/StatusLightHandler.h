#include <Arduino.h>
#include <Adafruit_DotStar.h>
#undef max
#undef min
#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"

#ifndef STATUS_LIGHT_HANDLER_H
#define STATUS_LIGHT_HANDLER_H

class StatusLightHandler
{
 public:
    StatusLightHandler();
    ~StatusLightHandler() = default;

    void showDisabled();
    void showEnabled();
    void showOpenLoop();
    void showCommunicationActive();
    void showCommunicationInactive();

 private:
    Adafruit_DotStar dotStarLed;
    uint8_t dotstarState;
    uint8_t dotstarStateRequest;
    uint8_t comActiveCounter;
    uint8_t comActiveBlinkCount;
    uint8_t comInactiveCounter;
    uint8_t comInactiveBlinkCount;

    uint8_t red;
    uint8_t green;
    uint8_t blue;

    uint8_t oldRedValue;
    uint8_t oldGreenValue;
    uint8_t oldBlueValue;

    std::vector<Thread*> threads;
};

#endif

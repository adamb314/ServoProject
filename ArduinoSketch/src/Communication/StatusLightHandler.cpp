#include "StatusLightHandler.h"

StatusLightHandler::StatusLightHandler() :
        dotStarLed(1, 41, 40, DOTSTAR_BGR),
        dotstarState(1),
        dotstarStateRequest(1),
        comActiveCounter(0),
        comActiveBlinkCount(0),
        comInactiveCounter(0),
        comInactiveBlinkCount(0)
{
    dotStarLed.begin();
    dotStarLed.show();

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);


    threads.push_back(createThread(0, 30000, 0,
        [&]()
        {
            if (comActiveBlinkCount + comInactiveBlinkCount > 0)
            {
                if (comActiveBlinkCount > comInactiveBlinkCount)
                {
                    switch (dotstarState)
                    {
                        case 1:
                            red = 50;
                            green = 30;
                            blue = 30;
                            break;

                        case 2:
                            red = 60;
                            green = 50;
                            blue = 60;
                            break;

                        case 3:
                            red = 30;
                            green = 30;
                            blue = 50;
                            break;
                    }
                }
                else
                {
                    red = 0;
                    green = 0;
                    blue = 0;
                }

                if (comActiveBlinkCount > 0)
                {
                    comActiveBlinkCount--;
                }

                if (comInactiveBlinkCount > 0)
                {
                    comInactiveBlinkCount--;
                }
            }
            else
            {
                switch (dotstarState)
                {
                    case 1:
                        red = 50;
                        green = 0;
                        blue = 0;

                        if (dotstarStateRequest != 1)
                        {
                            dotstarState = dotstarStateRequest;
                        }
                        break;

                    case 2:
                        red = 60;
                        green = 50;
                        blue = 0;

                        if (dotstarStateRequest != 2)
                        {
                            dotstarState = dotstarStateRequest;
                        }
                        break;

                    case 3:
                        red = 0;
                        green = 0;
                        blue = 50;

                        if (dotstarStateRequest != 3)
                        {
                            dotstarState = dotstarStateRequest;
                        }
                        break;

                    default:
                        dotstarState = dotstarStateRequest;
                        break;
                }
            }

            bool updateColor = (red != oldRedValue) || (green != oldGreenValue) || (blue != oldBlueValue);
            if (updateColor)
            {
                oldRedValue = red;
                oldGreenValue = green;
                oldBlueValue = blue;

                dotStarLed.setPixelColor(0, red, green, blue);
                dotStarLed.show();

                if ((static_cast<int>(red) + green + blue) != 0 &&
                    (static_cast<int>(red) * green * blue) == 0)
                {
                    digitalWrite(13, HIGH);
                }
                else
                {
                    digitalWrite(13, LOW);
                }
            }
        }));
}

void StatusLightHandler::showDisabled()
{
    dotstarStateRequest = 1;
}

void StatusLightHandler::showEnabled()
{
    dotstarStateRequest = 2;
}

void StatusLightHandler::showOpenLoop()
{
    dotstarStateRequest = 3;
}

void StatusLightHandler::showCommunicationActive()
{
    comInactiveCounter = 0;

    if (comActiveCounter == 0)
    {
        comActiveBlinkCount = 5;
    }

    comActiveCounter++;
    if (comActiveCounter > 200)
    {
        comActiveCounter = 0;
    }
}

void StatusLightHandler::showCommunicationInactive()
{
    comActiveCounter = 0;

    if (comInactiveCounter == 0)
    {
        comInactiveBlinkCount = 5;
    }

    comInactiveCounter++;
    if (comInactiveCounter > 10)
    {
        comInactiveCounter = 0;
    }
}

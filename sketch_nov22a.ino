#undef max
#undef min

#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "FilteredADC.h"
#include "EncoderHandler.h"
#include "PwmHandler.h"
#include "CurrentControlLoop.h"

START_OF_THREAD_HANDLER_CONFIG
THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(200);
END_OF_THREAD_HANDLER_CONFIG

#define PI_CONST (3.1415926535897932384626433832795028841972)

class ParameterParser
{
  public:
    ParameterParser()
    {
        state = 0;

        resetTimeoutTimer();

        amplitude = 200;
        frq = 4;
        testRunTimeMs = 1000;
    }

    ~ParameterParser()
    {
    }

    void resetTimeoutTimer()
    {
        timeAtLastRead = micros();
    }

    int parse(Serial_* serial)
    {
        if (static_cast<long>(micros() - timeAtLastRead) > 100000)
        {
            resetTimeoutTimer();
            state = 0;
            return -1;
        }

        switch (state)
        {
            case 0:
                if (serial->available())
                {
                    timeAtLastRead = micros();
                    parameter = serial->read();

                    switch (parameter)
                    {
                        case 'e':
                            return 1;

                        default:
                            if (isalpha(parameter))
                            {
                                state = 10;
                            }
                            break;
                    }
                }
                break;

            case 10:
                if (serial->available())
                {
                    timeAtLastRead = micros();

                    switch (parameter)
                    {
                        case 'a':
                            amplitude = serial->parseInt();
                            break;

                        case 'f':
                            frq = serial->parseInt();
                            break;

                        case 't':
                            testRunTimeMs = serial->parseInt();
                            break;

                        case 'r':
                            runTestNr = serial->parseInt();
                            break;

                        default:
                            break;
                    }
                    state = 0;
                }
                break;
        }

        return 0;
    }

    int amplitude;
    int frq;
    unsigned int testRunTimeMs;
    int runTestNr;

  private:
    int state;
    unsigned char parameter;
    unsigned long timeAtLastRead;
};

ThreadHandler* threadHandler = ThreadHandler::getInstance();

EncoderHandler* encoderHandler = nullptr;
CurrentControlLoop* currentControlLoop = nullptr;
unsigned long startTime;

Thread* testThread = nullptr;

int i = 0;
int testOutputArray[] = {-1, -1, 1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1, 1, -1, -1};
int testOutputArray2[] = {-1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, 1, -1, -1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1, -1, 1};
unsigned long dt = 200000;
int state = 0;
float t = 0;

ParameterParser param;

// the setup function runs once when you press reset or power the board
void setup()
{
    encoderHandler = new EncoderHandler();
    currentControlLoop = new CurrentControlLoop(400);
    Serial.begin(115200);
    encoderHandler->init();

    testThread = new FunctionThread(0, 1200, 0,
        [&]()
        {
            switch (state)
            {
                case 0:
                    currentControlLoop->overidePwmDuty(0);

                    if (param.parse(&Serial) == 1)
                    {
                        if (param.runTestNr == 1)
                        {
                            state = 10;
                        }
                        else if (param.runTestNr == 2)
                        {
                            state = 20;
                        }
                        else if (param.runTestNr == 3)
                        {
                            state = 30;
                        }
                    }
                    break;

                case 10:
                    startTime = micros();
                    dt = param.testRunTimeMs / 100 * 1000 / 16;
                    i = 0;

                    state = 11;

                case 11:
                    if (i / 16 >= sizeof(testOutputArray) / sizeof(testOutputArray[0]))
                    {
                        int pwm = 0;
                        currentControlLoop->overidePwmDuty(pwm);

                        Serial.print(pwm);
                        Serial.print(" ");
                        Serial.print(static_cast<int16_t>(currentControlLoop->getCurrent()));
                        Serial.print(" ");
                        encoderHandler->triggerSample();
                        Serial.println(encoderHandler->getValue());
                        state = 0;
                        break;
                    }

                    {
                        int pwm = param.amplitude * testOutputArray[i / 16];
                        ++i;
                        currentControlLoop->overidePwmDuty(pwm);

                        Serial.print(pwm);
                        Serial.print(" ");
                        Serial.print(static_cast<int16_t>(currentControlLoop->getCurrent()));
                        Serial.print(" ");
                    }

                    encoderHandler->triggerSample();
                    Serial.println(encoderHandler->getValue());

                    while (static_cast<long>(startTime - micros()) > 0)
                    {
                    }
                    startTime += dt;
                    
                    break;

                case 20:
                    startTime = micros();
                    dt = param.testRunTimeMs / 100 * 1000;
                    i = 0;

                    state = 21;

                case 21:
                    if (i >= sizeof(testOutputArray2) / sizeof(testOutputArray2[0]))
                    {
                        int pwm = 0;
                        currentControlLoop->overidePwmDuty(pwm);

                        Serial.print(pwm);
                        Serial.print(" ");
                        Serial.print(static_cast<int16_t>(currentControlLoop->getCurrent()));
                        Serial.print(" ");
                        encoderHandler->triggerSample();
                        Serial.println(encoderHandler->getValue());
                        state = 0;
                        break;
                    }

                    {
                        int pwm = param.amplitude * testOutputArray[i];
                        ++i;
                        currentControlLoop->overidePwmDuty(pwm);

                        Serial.print(pwm);
                        Serial.print(" ");
                        Serial.print(static_cast<int16_t>(currentControlLoop->getCurrent()));
                        Serial.print(" ");
                    }

                    encoderHandler->triggerSample();
                    Serial.println(encoderHandler->getValue());

                    while (static_cast<long>(startTime - micros()) > 0)
                    {
                    }
                    startTime += dt;

                    break;

                case 30:
                    param.parse(&Serial);
                    t = 0;
                    dt = 1200;
                    state = 31;

                case 31:
                    {
                        int pwm = static_cast<int>(roundf(param.amplitude * cos(t * 2 * PI_CONST * param.frq)));
                        if (pwm > 0.6 * param.amplitude)
                        {
                            pwm = param.amplitude;
                        }
                        else if (pwm < -0.6 * param.amplitude)
                        {
                            pwm = -param.amplitude;
                        }
                        else
                        {
                            pwm = 0;
                        }

                        t += 0.0012;

                        if (t * 1000 >= param.testRunTimeMs)
                        {
                            pwm = 0;
                            state = 0;
                        }

                        float current = currentControlLoop->getCurrent();
                        currentControlLoop->overidePwmDuty(pwm);

                        Serial.print(pwm);
                        Serial.print(" ");
                        Serial.print(static_cast<int16_t>(current));
                        Serial.print(" ");
                        encoderHandler->triggerSample();
                        Serial.println(encoderHandler->getValue());
                    }

                    break;
            }
        });
}

// the loop function runs over and over again forever
void loop()
{
    threadHandler->run();
}

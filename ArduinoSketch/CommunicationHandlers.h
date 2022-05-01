#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "StatusLightHandler.h"
#include "Communication.h"
#include "DCServo.h"
#include <Servo.h>

#ifndef COMMUNICATION_HANDLERS_H
#define COMMUNICATION_HANDLERS_H

#include <type_traits>

template <typename T, typename U>
class ContinuousValueUpCaster
{
  public:
    typedef typename std::decay<T>::type ValueType;
    typedef typename std::decay<U>::type InputType;

    const ValueType& get()
    {
        return value;
    }

    void set(const ValueType& v)
    {
        value = v;
    }

    void update(const InputType& input)
    {
        typedef typename std::make_signed<InputType>::type SignedInputType;

        SignedInputType diff = input - value;

        value += diff;
    }

  protected:
    ValueType value{0};
};

class DCServoCommunicationHandler : public CommunicationNode
{
public:
    DCServoCommunicationHandler(unsigned char nodeNr, std::unique_ptr<DCServo> dcServo);

    virtual ~DCServoCommunicationHandler();

    virtual void onReadyToSendEvent() override;

    virtual void onReceiveCompleteEvent() override;

    virtual void onErrorEvent() override;

    virtual void onComCycleEvent() override;

    virtual void onComIdleEvent() override;

protected:
    std::unique_ptr<DCServo> dcServo;

    StatusLightHandler statusLight;

    ContinuousValueUpCaster<long int, short int> intArrayIndex0Upscaler;

    static constexpr int positionUpscaling = 32;
};

class DCServoCommunicationHandlerWithPwmInterface : public DCServoCommunicationHandler
{
public:
    DCServoCommunicationHandlerWithPwmInterface(unsigned char nodeNr, std::unique_ptr<DCServo> dcServo, unsigned char pwmPin = 0,
            float scale = 2.0f, float offset = 1024.0f);

    virtual ~DCServoCommunicationHandlerWithPwmInterface()
    {
    }

    void onComIdleEvent() override
    {
    }

    virtual void comIdleRun() override;

protected:
    void config_tcc();

    /* Sense: 
     * None, Rise, Fall, Both, High, Low
     * 0x0   0x1   0x2   0x3   0x4   0x5
     */
    void config_eic_channel(int ch, int sense, bool filt);

    void config_eic();

    void config_evsys();

    void gpio_in(int port, int pin);

    void gpio_pmuxen(int port, int pin, int mux);

    void config_gpio();

    unsigned char pwmPin;
    float scale;
    float offset;
    uint8_t state{0};
    uint32_t lastPulseTime{0};
};

class ServoCommunicationHandler : public CommunicationNode
{
public:
    ServoCommunicationHandler(unsigned char nodeNr, unsigned char servoPin) :
        CommunicationNode(nodeNr),
        servoPin(servoPin)
    {
    }

    ~ServoCommunicationHandler()
    {
    }

    virtual void onReadyToSendEvent() override
    {
    }

    virtual void onReceiveCompleteEvent() override
    {
    }

    virtual void onErrorEvent() override
    {
    }

    virtual void onComCycleEvent() override
    {

        if (CommunicationNode::intArrayChanged[0])
        {
            CommunicationNode::intArrayChanged[0] = false;

            if (!servo.attached())
            {
                servo.attach(servoPin, 400, 2600);
            }

            int pos = static_cast<int>(CommunicationNode::intArray[0]) / 32;

            servo.writeMicroseconds(pos + 1500);

            CommunicationNode::intArray[3] = CommunicationNode::intArray[0];
        }
        else
        {
            servo.detach();
        }
    }

    virtual void onComIdleEvent() override
    {
        servo.detach();
    }

private:
    unsigned char servoPin;
    Servo servo;
};

#endif

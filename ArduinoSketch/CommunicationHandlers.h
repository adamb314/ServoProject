#include "ArduinoC++BugFixes.h"
#include "ThreadHandler.h"
#include "StatusLightHandler.h"
#include "Communication.h"
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

protected:
    StatusLightHandler statusLight;

    ContinuousValueUpCaster<long int, short int> intArrayIndex0Upscaler;

    static constexpr int positionUpscaling = 32;
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

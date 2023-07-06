#include "PwmHandler.h"

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(bool invert, LinearizeFunctionType linearizeFunction, uint16_t frq) :
    HBridgeHighResPin11And12Pwm(TCC0, invert, false, linearizeFunction, frq)
{
    connectOutput();
}

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(bool invert, bool dualSlope,
        LinearizeFunctionType linearizeFunction, uint16_t frq) :
    HBridgeHighResPin11And12Pwm(TCC0, invert, dualSlope, linearizeFunction, frq)
{
    connectOutput();
}

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(Tcc* timer, bool invert, bool dualSlope, 
        LinearizeFunctionType linearizeFunction, uint16_t frq) :
    timer(timer),
    invert(invert),
    dualSlope(dualSlope),
    linearizeFunction(linearizeFunction),
    freqDiv(static_cast<uint16_t>(F_CPU / 1024.0f / frq + 0.5f))
{
}

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(HBridgeHighResPin11And12Pwm&& in) :
    timer(in.timer),
    pin11WriteValue(in.pin11WriteValue),
    pin12WriteValue(in.pin12WriteValue),
    outputConnected(in.outputConnected),
    invert(in.invert),
    dualSlope(in.dualSlope),
    linearizeFunction(in.linearizeFunction)
{
    in.outputConnected = false;
}

HBridgeHighResPin11And12Pwm::~HBridgeHighResPin11And12Pwm()
{
    disconnectOutput();
}

int HBridgeHighResPin11And12Pwm::setOutput(int output)
{
    if (invert)
    {
        output = -output;
    }

    const int16_t maxPwm = 1023;

    if (output > maxPwm)
    {
        output = maxPwm;
    }
    else if (output < -maxPwm)
    {
        output = -maxPwm;
    }

    if (output >= 0)
    {
        pin11WriteValue = 0;
        auto temp = linearizeFunction(output);
        pin12WriteValue = freqDiv * temp + ((freqDiv - 1) * temp + 512) / 1024;
    }
    else
    {
        auto temp = linearizeFunction(-output);
        pin11WriteValue = freqDiv * temp + ((freqDiv - 1) * temp + 512) / 1024;
        pin12WriteValue = 0;
    }

    if (outputConnected)
    {
        while (timer->SYNCBUSY.bit.CTRLB == 1)
        {
        }
        timer->CTRLBSET.reg = TCC_CTRLBCLR_LUPD;
        while (timer->SYNCBUSY.bit.CTRLB == 1)
        {
        }

        timer->CCB[0].bit.CCB = pin11WriteValue;
        timer->CCB[1].bit.CCB = pin12WriteValue;

        timer->CTRLBCLR.reg = TCC_CTRLBCLR_LUPD;
    }

    if (invert)
    {
        return -output;
    }

    return output;
}

void HBridgeHighResPin11And12Pwm::activateBrake()
{
    pin11WriteValue = freqDiv * 1024 - 1;
    pin12WriteValue = freqDiv * 1024 - 1;

    if (outputConnected)
    {
        while (timer->SYNCBUSY.bit.CTRLB == 1)
        {
        }
        timer->CTRLBSET.reg = TCC_CTRLBCLR_LUPD;
        while (timer->SYNCBUSY.bit.CTRLB == 1)
        {
        }

        timer->CCB[0].bit.CCB = pin11WriteValue;
        timer->CCB[1].bit.CCB = pin12WriteValue;

        timer->CTRLBCLR.reg = TCC_CTRLBCLR_LUPD;
    }

    return;
}

void HBridgeHighResPin11And12Pwm::disconnectOutput()
{
    if (!outputConnected)
    {
        return;
    }

    while (timer->SYNCBUSY.bit.ENABLE == 1)
    {
    }

    timer->CTRLA.bit.ENABLE = false;

    while (timer->SYNCBUSY.bit.ENABLE == 1)
    {
    }

    timer->CC[0].bit.CC = 0;
    timer->CC[1].bit.CC = 0;

    outputConnected = false;
}

void HBridgeHighResPin11And12Pwm::connectOutput()
{
    if (outputConnected)
    {
        return;
    }

    PM->APBCMASK.bit.TCC0_ = true;
    
    configTimer();

    // Setting D11(PA16) and D12(PA19) as output
    PORT->Group[0].DIRCLR.reg = (1ul << 16) |
                             (1ul << 19);

    PORT->Group[0].OUTCLR.reg = (1ul << 16) |
                              (1ul << 19);

    PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG |
                                  PORT_WRCONFIG_WRPMUX |
                                  PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_F_Val) |
                                  PORT_WRCONFIG_PMUXEN |
                                  PORT_WRCONFIG_HWSEL |
                                  PORT_WRCONFIG_PINMASK(
                                    (1 << 0) |
                                    (1 << 3));

    outputConnected = true;
}

bool HBridgeHighResPin11And12Pwm::willSwitchWithIn(int16_t period) const
{
    if (timer->CTRLA.bit.ENABLE == false)
    {
        return false;
    }

    timer->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;

    int minPeriodCount;
    int maxPeriodCount;

    period *= tickPerUs;

    if (period >= 0)
    {
        minPeriodCount = 0;
        maxPeriodCount = period;
    }
    else
    {
        minPeriodCount = period;
        maxPeriodCount = 0;
    }

    auto maxCC = timer->CC[0].bit.CC;
    if (maxCC < timer->CC[1].bit.CC)
    {
        maxCC = timer->CC[1].bit.CC;
    }

    while (timer->CTRLBSET.bit.CMD)
    {
    }
    minPeriodCount += timer->COUNT.bit.COUNT;
    maxPeriodCount += timer->COUNT.bit.COUNT;

    bool switchInPeriod = minPeriodCount < maxCC + switchTransientTime;
    if (dualSlope)
    {
        switchInPeriod &= maxPeriodCount + switchTransientTime > maxCC;
    }
    else
    {
        switchInPeriod &= maxPeriodCount > maxCC;
        switchInPeriod |= maxPeriodCount > freqDiv * 1024 || minPeriodCount < switchTransientTime;
    }

    return switchInPeriod;
}

void HBridgeHighResPin11And12Pwm::configTimer()
{
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_TCC0_TCC1;

    while (GCLK->STATUS.bit.SYNCBUSY == 1)
    {
    }

    timer->CTRLA.bit.ENABLE = false;

    timer->CTRLA.reg = TCC_CTRLA_RESETVALUE |
                      TCC_CTRLA_RUNSTDBY |
                      TCC_CTRLA_PRESCALER_DIV1;

    timer->CTRLBCLR.reg = TCC_CTRLBCLR_RESETVALUE |
                         TCC_CTRLBCLR_DIR |
                         TCC_CTRLBCLR_LUPD |
                         TCC_CTRLBCLR_ONESHOT;

    timer->FCTRLA.reg = TCC_FCTRLA_RESETVALUE;
    timer->FCTRLB.reg = TCC_FCTRLB_RESETVALUE;

    timer->WEXCTRL.reg = TCC_WEXCTRL_RESETVALUE |
                        TCC_WEXCTRL_OTMX(0x1);

    timer->DRVCTRL.reg = TCC_DRVCTRL_RESETVALUE;

    timer->DBGCTRL.reg = TCC_DBGCTRL_RESETVALUE |
                        TCC_DBGCTRL_DBGRUN;

    auto temp = TCC_WAVE_RESETVALUE;
    if (dualSlope)
    {
        temp |= TCC_WAVE_WAVEGEN_DSBOTH | TCC_WAVE_POL(0xf);

    }
    else
    {
        temp |= TCC_WAVE_WAVEGEN_NPWM;
    }
    timer->WAVE.reg = temp;

    timer->PATT.reg = TCC_PATT_RESETVALUE;

    timer->PER.bit.PER = freqDiv * 1024;

    timer->CC[0].bit.CC = pin11WriteValue;
    timer->CC[1].bit.CC = pin12WriteValue;

    timer->WAVEB.reg = timer->WAVE.reg;

    timer->PATTB.reg = timer->PATT.reg;

    timer->PERB.bit.PERB = timer->PER.bit.PER;

    timer->CCB[0].bit.CCB = timer->CC[0].bit.CC;
    timer->CCB[1].bit.CCB = timer->CC[1].bit.CC;

    timer->CTRLA.bit.ENABLE = true;
}

HBridgeHighResPin3And4Pwm::HBridgeHighResPin3And4Pwm(bool invert, LinearizeFunctionType linearizeFunction, uint16_t frq) :
    HBridgeHighResPin11And12Pwm(TCC1, invert, false, linearizeFunction, frq)
{
    connectOutput();
}

HBridgeHighResPin3And4Pwm::HBridgeHighResPin3And4Pwm(bool invert, bool dualSlope,
        LinearizeFunctionType linearizeFunction, uint16_t frq) :
    HBridgeHighResPin11And12Pwm(TCC0, invert, dualSlope, linearizeFunction, frq)
{
    connectOutput();
}

HBridgeHighResPin3And4Pwm::HBridgeHighResPin3And4Pwm(HBridgeHighResPin3And4Pwm&& in) :
    HBridgeHighResPin11And12Pwm(std::move(in))
{
}

void HBridgeHighResPin3And4Pwm::connectOutput()
{
    if (outputConnected)
    {
        return;
    }

    PM->APBCMASK.bit.TCC1_ = true;
    
    configTimer();

    // Setting D3(PA9) and D4(PA8) as output
    PORT->Group[0].DIRCLR.reg = (1ul << 8) |
                             (1ul << 9);

    PORT->Group[0].OUTCLR.reg = (1ul << 8) |
                              (1ul << 9);

    PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG |
                                  PORT_WRCONFIG_WRPMUX |
                                  PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_F_Val) |
                                  PORT_WRCONFIG_PMUXEN |
                                  PORT_WRCONFIG_PINMASK(
                                    (1 << 8) |
                                    (1 << 9));

    outputConnected = true;
}

HBridge2WirePwm::HBridge2WirePwm(int16_t pin1, int16_t pin2, LinearizeFunctionType linearizeFunction) :
    pin1(pin1), pin2(pin2), linearizeFunction(linearizeFunction)
{
    connectOutput();
}

HBridge2WirePwm::HBridge2WirePwm(HBridge2WirePwm&& in) :
    pin1(in.pin1), pin2(in.pin2),
    pin1WriteValue(in.pin1WriteValue),
    pin2WriteValue(in.pin2WriteValue),
    outputConnected(in.outputConnected),
    linearizeFunction(in.linearizeFunction)
{
    in.pin1 = -1;
    in.pin2 = -1;
    in.outputConnected = false;
}

HBridge2WirePwm::~HBridge2WirePwm()
{
    disconnectOutput();
}

int HBridge2WirePwm::setOutput(int output)
{
    const int16_t maxPwm = 1023;

    if (output > maxPwm)
    {
        output = maxPwm;
    }
    else if (output < -maxPwm)
    {
        output = -maxPwm;
    }

    if (output >= 0)
    {
        pin1WriteValue = 0;
        pin2WriteValue = linearizeFunction(output) / 4;
    }
    else
    {
        pin1WriteValue = linearizeFunction(-output) / 4;
        pin2WriteValue = 0;
    }

    if (outputConnected)
    {
        analogWrite(pin1, pin1WriteValue);
        analogWrite(pin2, pin2WriteValue);
    }

    return output;
}

void HBridge2WirePwm::activateBrake()
{
    if (pin1WriteValue != 255 ||
        pin2WriteValue != 255)
    {
        analogWrite(pin1, 0);
        analogWrite(pin2, 0);
    }

    pin1WriteValue = 255;
    pin2WriteValue = 255;

    if (outputConnected)
    {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, HIGH);
    }

    return;
}

void HBridge2WirePwm::disconnectOutput()
{
    if (!outputConnected)
    {
        return;
    }

    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);

    outputConnected = false;
}

void HBridge2WirePwm::connectOutput()
{
    if (outputConnected || pin1 == -1 || pin2 == -1)
    {
        return;
    }

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);

    if (pin1WriteValue == 255 &&
        pin2WriteValue == 255)
    {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, HIGH);
    }
    else
    {
        analogWrite(pin1, pin1WriteValue);
        analogWrite(pin2, pin2WriteValue);
    }

    outputConnected = true;
}

SwitchAvoidingSynchronizer::Switcher::~Switcher()
{
    while(synchronizers.size() != 0)
    {
        bool removeCompleted = synchronizers.back()->removeSwitcher(this);
        if (removeCompleted == false)
        {
            removeSynchronizer(synchronizers.back());
        }
    }
}

void SwitchAvoidingSynchronizer::Switcher::addSynchronizer(SwitchAvoidingSynchronizer* synchronizer)
{
    synchronizers.push_back(synchronizer);
}

void SwitchAvoidingSynchronizer::Switcher::removeSynchronizer(SwitchAvoidingSynchronizer* synchronizer)
{
    synchronizers.erase(std::remove(std::begin(synchronizers), std::end(synchronizers), synchronizer),
        std::end(synchronizers));
}

SwitchAvoidingSynchronizer::~SwitchAvoidingSynchronizer()
{
    std::for_each(std::begin(switchers), std::end(switchers), [this](Switcher* switcher)
            {
                switcher->removeSynchronizer(this);
            });
    switchers.clear();
}

void SwitchAvoidingSynchronizer::addSwitcher(SwitchAvoidingSynchronizer::Switcher* switcher)
{
    if (std::count(std::begin(switchers), std::end(switchers), switcher) != 0)
    {
        return;
    }

    switchers.push_back(switcher);
    switcher->addSynchronizer(this);
}

bool SwitchAvoidingSynchronizer::removeSwitcher(SwitchAvoidingSynchronizer::Switcher* switcher)
{
    auto it = std::find(std::begin(switchers), std::end(switchers), switcher);
    if (it != std::end(switchers))
    {
        switchers.erase(it);
        switcher->removeSynchronizer(this);
        return true;
    }
    return false;
}

bool SwitchAvoidingSynchronizer::willSwitchWithIn(int16_t period)
{
    for (auto it = std::begin(switchers); it != std::end(switchers); ++it)
    {
        if ((*it)->willSwitchWithIn(period))
        {
            return true;
        }
    }
    return false;
}

SwitchAvoidingSumAnalogSampler::SwitchAvoidingSumAnalogSampler(uint32_t pin,
            std::shared_ptr<SwitchAvoidingSynchronizer> synchronizer, uint16_t numberOfAdditiveSamples) :
    AdcSamplerInstance(pin), numberOfAdditiveSamples(numberOfAdditiveSamples),
    synchronizer(synchronizer)
{
}

SwitchAvoidingSumAnalogSampler::~SwitchAvoidingSumAnalogSampler()
{
}

void SwitchAvoidingSumAnalogSampler::triggerSample()
{
    AdcSamplerInstance::getAdcLockAndStartSampling();
}

int32_t SwitchAvoidingSumAnalogSampler::getValue()
{
    AdcSamplerInstance::unlockFromAdc();
    return value;
}

void SwitchAvoidingSumAnalogSampler::loadConfigAndStart()
{
    value = 0;
    sumOfAllSamples = 0;
    samplesLeft = numberOfAdditiveSamples;
    switchFreeSamplesTaken = 0;
    AdcSamplerInstance::startAdcSample();
}

bool SwitchAvoidingSumAnalogSampler::handleResultAndCleanUp(int32_t result)
{
    sumOfAllSamples += result;
    --samplesLeft;
    if (!synchronizer->willSwitchWithIn(-13))
    {
        value += result;
        ++switchFreeSamplesTaken;
    }

    if (samplesLeft == 0)
    {
        if (switchFreeSamplesTaken != 0)
        {
            value *= numberOfAdditiveSamples;
            value /= switchFreeSamplesTaken;
        }
        else
        {
            value = sumOfAllSamples;
        }
        return true;
    }

    return false;
}

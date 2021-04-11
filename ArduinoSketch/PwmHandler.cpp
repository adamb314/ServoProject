#include "PwmHandler.h"

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(bool invert, LinearizeFunctionType linearizeFunction) :
    timer(TCC0),
    invert(invert),
    linearizeFunction(linearizeFunction)
{
    connectOutput();
}

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(Tcc* timer, bool invert, LinearizeFunctionType linearizeFunction) :
    timer(timer),
    invert(invert),
    linearizeFunction(linearizeFunction)
{
}

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(HBridgeHighResPin11And12Pwm&& in) :
    timer(in.timer),
    pin11WriteValue(in.pin11WriteValue),
    pin12WriteValue(in.pin12WriteValue),
    outputConnected(in.outputConnected),
    invert(in.invert),
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
        pin12WriteValue = 2 * linearizeFunction(output);
    }
    else
    {
        pin11WriteValue = 2 * linearizeFunction(-output);
        pin12WriteValue = 0;
    }

    if (outputConnected)
    {
        timer->CTRLBSET.reg = TCC_CTRLBCLR_RESETVALUE |
                             TCC_CTRLBCLR_LUPD;

        timer->CCB[0].bit.CCB = pin11WriteValue;
        timer->CCB[1].bit.CCB = pin12WriteValue;

        timer->CTRLBCLR.reg = TCC_CTRLBCLR_RESETVALUE |
                             TCC_CTRLBCLR_LUPD;
    }

    if (invert)
    {
        return -output;
    }

    return output;
}

void HBridgeHighResPin11And12Pwm::activateBrake()
{
    pin11WriteValue = 2 * 1023;
    pin12WriteValue = 2 * 1023;

    if (outputConnected)
    {
        timer->CTRLBSET.reg = TCC_CTRLBCLR_RESETVALUE |
                             TCC_CTRLBCLR_LUPD;

        timer->CCB[0].bit.CCB = pin11WriteValue;
        timer->CCB[1].bit.CCB = pin12WriteValue;

        timer->CTRLBCLR.reg = TCC_CTRLBCLR_RESETVALUE |
                             TCC_CTRLBCLR_LUPD;
    }

    return;
}

void HBridgeHighResPin11And12Pwm::disconnectOutput()
{
    if (!outputConnected)
    {
        return;
    }

    timer->CTRLA.bit.ENABLE = false;

    // sync

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

    timer->WAVE.reg = TCC_WAVE_RESETVALUE |
                     TCC_WAVE_WAVEGEN_NPWM;

    timer->PATT.reg = TCC_PATT_RESETVALUE;

    timer->PER.bit.PER = 2 * 1024;

    timer->CC[0].bit.CC = pin11WriteValue;
    timer->CC[1].bit.CC = pin12WriteValue;

    timer->WAVEB.reg = timer->WAVE.reg;

    timer->PATTB.reg = timer->PATT.reg;

    timer->PERB.bit.PERB = timer->PER.bit.PER;

    timer->CCB[0].bit.CCB = timer->CC[0].bit.CC;
    timer->CCB[1].bit.CCB = timer->CC[1].bit.CC;

    timer->CTRLA.bit.ENABLE = true;
}

HBridgeHighResPin3And4Pwm::HBridgeHighResPin3And4Pwm(bool invert, LinearizeFunctionType linearizeFunction) :
    HBridgeHighResPin11And12Pwm(TCC1, invert, linearizeFunction)
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

#include "PwmHandler.h"

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm()
{
    connectOutput();
}

HBridgeHighResPin11And12Pwm::HBridgeHighResPin11And12Pwm(HBridgeHighResPin11And12Pwm&& in) :
    pin11WriteValue(in.pin11WriteValue),
    pin12WriteValue(in.pin12WriteValue),
    outputConnected(in.outputConnected)
{
    in.outputConnected = false;
}

HBridgeHighResPin11And12Pwm::~HBridgeHighResPin11And12Pwm()
{
    disconnectOutput();
}

int HBridgeHighResPin11And12Pwm::setOutput(int output)
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
        pin11WriteValue = 0;
        pin12WriteValue = 2 * output;
    }
    else
    {
        pin11WriteValue = -2 * output;
        pin12WriteValue = 0;
    }

    if (outputConnected)
    {
        TCC0->CTRLBSET.reg = TCC_CTRLBCLR_RESETVALUE |
                             TCC_CTRLBCLR_LUPD;

        TCC0->CCB[0].bit.CCB = pin11WriteValue;
        TCC0->CCB[1].bit.CCB = pin12WriteValue;

        TCC0->CTRLBCLR.reg = TCC_CTRLBCLR_RESETVALUE |
                             TCC_CTRLBCLR_LUPD;
    }

    return output;
}

void HBridgeHighResPin11And12Pwm::activateBrake()
{
    pin11WriteValue = 2 * 1023;
    pin12WriteValue = 2 * 1023;

    if (outputConnected)
    {
        TCC0->CTRLBSET.reg = TCC_CTRLBCLR_RESETVALUE |
                             TCC_CTRLBCLR_LUPD;

        TCC0->CCB[0].bit.CCB = pin11WriteValue;
        TCC0->CCB[1].bit.CCB = pin12WriteValue;

        TCC0->CTRLBCLR.reg = TCC_CTRLBCLR_RESETVALUE |
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

    TCC0->CTRLA.bit.ENABLE = false;

    // sync

    TCC0->CC[0].bit.CC = 0;
    TCC0->CC[1].bit.CC = 0;

    outputConnected = false;
}

void HBridgeHighResPin11And12Pwm::connectOutput()
{
    if (outputConnected)
    {
        return;
    }

    PM->APBCMASK.bit.TCC0_ = true;

    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_ID_TCC0_TCC1;

    while (GCLK->STATUS.bit.SYNCBUSY == 1)
    {
    }

    TCC0->CTRLA.bit.ENABLE = false;

    TCC0->CTRLA.reg = TCC_CTRLA_RESETVALUE |
                      TCC_CTRLA_RUNSTDBY |
                      TCC_CTRLA_PRESCALER_DIV1;

    TCC0->CTRLBCLR.reg = TCC_CTRLBCLR_RESETVALUE |
                         TCC_CTRLBCLR_DIR |
                         TCC_CTRLBCLR_LUPD |
                         TCC_CTRLBCLR_ONESHOT;

    TCC0->FCTRLA.reg = TCC_FCTRLA_RESETVALUE;
    TCC0->FCTRLB.reg = TCC_FCTRLB_RESETVALUE;

    TCC0->WEXCTRL.reg = TCC_WEXCTRL_RESETVALUE |
                        TCC_WEXCTRL_OTMX(0x1);

    // Inverts output on D2 and D5
    TCC0->DRVCTRL.reg = TCC_DRVCTRL_RESETVALUE |
                        TCC_DRVCTRL_INVEN4 |
                        TCC_DRVCTRL_INVEN5;

    TCC0->DBGCTRL.reg = TCC_DBGCTRL_RESETVALUE |
                        TCC_DBGCTRL_DBGRUN;

    TCC0->WAVE.reg = TCC_WAVE_RESETVALUE |
                     TCC_WAVE_WAVEGEN_NPWM;

    TCC0->PATT.reg = TCC_PATT_RESETVALUE;

    TCC0->PER.bit.PER = 2 * 1024;

    TCC0->CC[0].bit.CC = pin11WriteValue;
    TCC0->CC[1].bit.CC = pin12WriteValue;

    TCC0->CC[2].bit.CC = 0;
    TCC0->CC[3].bit.CC = 0;

    TCC0->WAVEB.reg = TCC0->WAVE.reg;

    TCC0->PATTB.reg = TCC0->PATT.reg;

    TCC0->PERB.bit.PERB = TCC0->PER.bit.PER;

    TCC0->CCB[0].bit.CCB = TCC0->CC[0].bit.CC;
    TCC0->CCB[1].bit.CCB = TCC0->CC[1].bit.CC;
    TCC0->CCB[2].bit.CCB = TCC0->CC[2].bit.CC;
    TCC0->CCB[3].bit.CCB = TCC0->CC[3].bit.CC;

    TCC0->CTRLA.bit.ENABLE = true;

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

HBridge2WirePwm::HBridge2WirePwm(int16_t pin1, int16_t pin2) :
    pin1(pin1), pin2(pin2)
{
    connectOutput();
}

HBridge2WirePwm::HBridge2WirePwm(HBridge2WirePwm&& in) :
    pin1(in.pin1), pin2(in.pin2),
    pin1WriteValue(in.pin1WriteValue),
    pin2WriteValue(in.pin2WriteValue),
    outputConnected(in.outputConnected)
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
        pin2WriteValue = output / 4;
    }
    else
    {
        pin1WriteValue = (-output) / 4;
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
    pin1WriteValue = 255;
    pin2WriteValue = 255;

    if (outputConnected)
    {
        analogWrite(pin1, pin1WriteValue);
        analogWrite(pin2, pin2WriteValue);
    }

    return;
}

void HBridge2WirePwm::disconnectOutput()
{
    if (!outputConnected)
    {
        return;
    }

    analogWrite(pin1, 0);
    analogWrite(pin2, 0);

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

    analogWrite(pin1, pin1WriteValue);
    analogWrite(pin2, pin2WriteValue);

    outputConnected = true;
}

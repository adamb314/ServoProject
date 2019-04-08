#include "PwmHandler.h"

HBridge4WirePwm* HBridge4WirePwm::getInstance()
{
    static HBridge4WirePwm instance;

    return &instance;
}

HBridge4WirePwm::~HBridge4WirePwm()
{
    disconnectOutput();
}

int HBridge4WirePwm::setOutput(int output)
{
    if (output > 1023)
    {
        output = 1023;
    }
    else if (output < -1023)
    {
        output = -1023;
    }

    TCC0->CTRLBSET.reg = TCC_CTRLBCLR_RESETVALUE |
                         TCC_CTRLBCLR_LUPD;

    if (output >= 0)
    {
        TCC0->CCB[0].bit.CCB = 0;
        TCC0->CCB[1].bit.CCB = 2 * output;
    }
    else
    {
        TCC0->CCB[0].bit.CCB = -2 * output;
        TCC0->CCB[1].bit.CCB = 0;
    }

    TCC0->CTRLBCLR.reg = TCC_CTRLBCLR_RESETVALUE |
                         TCC_CTRLBCLR_LUPD;

    return output;
}

void HBridge4WirePwm::disconnectOutput()
{
    TCC0->CTRLA.bit.ENABLE = false;

    // sync

    disconnectOutputCC0Backup = TCC0->CC[0].bit.CC;
    disconnectOutputCC1Backup = TCC0->CC[1].bit.CC;

    TCC0->CC[0].bit.CC = 0;
    TCC0->CC[1].bit.CC = 0;
}

void HBridge4WirePwm::connectOutput()
{
    if (TCC0->CTRLA.bit.ENABLE == true)
    {
        return;
    }

    TCC0->CC[0].bit.CC = disconnectOutputCC0Backup;
    TCC0->CC[1].bit.CC = disconnectOutputCC1Backup;

    // sync

    TCC0->CTRLA.bit.ENABLE = true;

    // sync
}

HBridge4WirePwm::HBridge4WirePwm()
{
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

    TCC0->CC[0].bit.CC = 0;
    TCC0->CC[1].bit.CC = 0;
    TCC0->CC[2].bit.CC = 0;
    TCC0->CC[3].bit.CC = 0;

    disconnectOutputCC0Backup = TCC0->CC[0].bit.CC;
    disconnectOutputCC1Backup = TCC0->CC[1].bit.CC;

    TCC0->WAVEB.reg = TCC0->WAVE.reg;

    TCC0->PATTB.reg = TCC0->PATT.reg;

    TCC0->PERB.bit.PERB = TCC0->PER.bit.PER;

    TCC0->CCB[0].bit.CCB = TCC0->CC[0].bit.CC;
    TCC0->CCB[1].bit.CCB = TCC0->CC[1].bit.CC;
    TCC0->CCB[2].bit.CCB = TCC0->CC[2].bit.CC;
    TCC0->CCB[3].bit.CCB = TCC0->CC[3].bit.CC;

    TCC0->CTRLA.bit.ENABLE = true;

    // Setting D2 (PA14), D5 (PA15), D11(PA16), D12(PA19) and D13(PA17) as output
    PORT->Group[0].DIRCLR.reg = (1ul << 14) |
                             (1ul << 15) |
                             (1ul << 16) |
                             (1ul << 17) |
                             (1ul << 19);

    PORT->Group[0].OUTCLR.reg = (1ul << 14) |
                              (1ul << 15) |
                              (1ul << 16) |
                              (1ul << 17) |
                              (1ul << 19);

    PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG |
                                  PORT_WRCONFIG_WRPMUX |
                                  PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_F_Val) |
                                  PORT_WRCONFIG_PMUXEN |
                                  PORT_WRCONFIG_PINMASK(
                                    (1ul << 14) | (1ul << 15));

    PORT->Group[0].WRCONFIG.reg = PORT_WRCONFIG_WRPINCFG |
                                  PORT_WRCONFIG_WRPMUX |
                                  PORT_WRCONFIG_PMUX(PORT_PMUX_PMUXE_F_Val) |
                                  PORT_WRCONFIG_PMUXEN |
                                  PORT_WRCONFIG_HWSEL |
                                  PORT_WRCONFIG_PINMASK(
                                    (1 << 0) | (1 << 1) |
                                    (1 << 3));
}

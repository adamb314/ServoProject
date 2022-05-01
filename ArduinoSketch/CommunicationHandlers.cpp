#include "CommunicationHandlers.h"

static ThreadHandler* threadHandler = nullptr;

DCServoCommunicationHandler::DCServoCommunicationHandler(unsigned char nodeNr, std::unique_ptr<DCServo> dcServo) :
    CommunicationNode(nodeNr), dcServo(std::move(dcServo))
{
    threadHandler = ThreadHandler::getInstance();
    CommunicationNode::intArray[0] = this->dcServo->getPosition() * positionUpscaling;
    CommunicationNode::intArray[1] = 0;
    CommunicationNode::intArray[2] = 0;

    intArrayIndex0Upscaler.set(CommunicationNode::intArray[0]);
}

DCServoCommunicationHandler::~DCServoCommunicationHandler()
{
}

void DCServoCommunicationHandler::onReadyToSendEvent()
{
}

void DCServoCommunicationHandler::onReceiveCompleteEvent()
{
    dcServo->onlyUseMainEncoder(CommunicationNode::charArray[2] == 1);

    if (CommunicationNode::charArrayChanged[3] ||
        CommunicationNode::charArrayChanged[4] ||
        CommunicationNode::charArrayChanged[5])
    {
        if (!CommunicationNode::charArrayChanged[4] &&
                !CommunicationNode::charArrayChanged[5])
        {
            CommunicationNode::charArray[4] = CommunicationNode::charArray[3];
            CommunicationNode::charArray[5] = CommunicationNode::charArray[3];
        }
        dcServo->setControlSpeed(CommunicationNode::charArray[3],
                CommunicationNode::charArray[4] * 4,
                CommunicationNode::charArray[5] * 32);
    }

    if (CommunicationNode::charArrayChanged[6] ||
        CommunicationNode::charArrayChanged[7] ||
        CommunicationNode::charArrayChanged[8])
    {
        dcServo->setBacklashControlSpeed(CommunicationNode::charArray[6],
                CommunicationNode::charArray[7],
                CommunicationNode::charArray[8]);
    }

    if (CommunicationNode::intArrayChanged[0])
    {
        intArrayIndex0Upscaler.update(CommunicationNode::intArray[0]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0f / positionUpscaling), CommunicationNode::intArray[1], CommunicationNode::intArray[2]);

        CommunicationNode::intArrayChanged[0] = false;
        dcServo->openLoopMode(false);
        dcServo->enable(true);

        statusLight.showEnabled();
    }
    else if (CommunicationNode::intArrayChanged[2])
    {
        intArrayIndex0Upscaler.set(CommunicationNode::intArray[3]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0f / positionUpscaling), 0.0f, CommunicationNode::intArray[2]);

        CommunicationNode::intArrayChanged[2] = false;
        dcServo->openLoopMode(true, CommunicationNode::charArray[1] == 1);
        dcServo->enable(true);

        statusLight.showOpenLoop();
    }
    else
    {
        dcServo->enable(false);

        intArrayIndex0Upscaler.set(CommunicationNode::intArray[3]);

        statusLight.showDisabled();
    }
}

void DCServoCommunicationHandler::onErrorEvent()
{
}

void DCServoCommunicationHandler::onComCycleEvent()
{
    {
        ThreadInterruptBlocker blocker;

        long int pos = dcServo->getPosition() * positionUpscaling;
        CommunicationNode::intArray[3] = pos;
        CommunicationNode::charArray[9] = static_cast<char>(pos >> 16);
        CommunicationNode::intArray[4] = dcServo->getVelocity();
        CommunicationNode::intArray[5] = dcServo->getControlSignal();
        CommunicationNode::intArray[6] = dcServo->getCurrent();
        CommunicationNode::intArray[7] = dcServo->getPwmControlSignal();
        CommunicationNode::intArray[8] = threadHandler->getCpuLoad();
        CommunicationNode::intArray[9] = dcServo->getLoopTime();
        CommunicationNode::intArray[10] = dcServo->getMainEncoderPosition() * positionUpscaling;
        CommunicationNode::intArray[11] = dcServo->getBacklashCompensation() * positionUpscaling;

        auto opticalEncoderChannelData = dcServo->getMainEncoderDiagnosticData();
        CommunicationNode::intArray[12] = opticalEncoderChannelData.a;
        CommunicationNode::intArray[13] = opticalEncoderChannelData.b;
        CommunicationNode::intArray[14] = opticalEncoderChannelData.c;
        CommunicationNode::intArray[15] = opticalEncoderChannelData.d;

        dcServo->triggerReferenceTiming();
    }

    statusLight.showCommunicationActive();
}

void DCServoCommunicationHandler::onComIdleEvent()
{
    CommunicationNode::intArrayChanged[0] = false;
    CommunicationNode::intArrayChanged[2] = false;
    dcServo->enable(false);

    statusLight.showDisabled();
    statusLight.showCommunicationInactive();
}

DCServoCommunicationHandlerWithPwmInterface::
        DCServoCommunicationHandlerWithPwmInterface(unsigned char nodeNr, std::unique_ptr<DCServo> dcServo, unsigned char pwmPin,
            float scale, float offset) :
    DCServoCommunicationHandler(nodeNr, std::move(dcServo)),
    pwmPin(pwmPin), scale(scale * (1.0f / 24)), offset(offset)
{
    pinMode(pwmPin, INPUT_PULLUP);
    config_tcc();    // Configure the timer
    config_eic();    // Configure the external interruption
    config_evsys();  // Configure the event system
    config_gpio();   // Configure the dedicated pin
}

void DCServoCommunicationHandlerWithPwmInterface::comIdleRun()
{
    uint32_t time = micros();

    uint16_t newPulseLenth = 0;

    if (TCC2->CC[1].bit.CC != 0)
    {
        TCC2->CC[1].bit.CC = 0;
        if (TCC2->CC[0].bit.CC != 0)
        {
            TCC2->CC[0].bit.CC = 0;

            state = 0;
            return;
        }

        DCServoCommunicationHandler::dcServo->triggerReferenceTiming();
        statusLight.showCommunicationActive();

        lastPulseTime = time;
    }

    if (TCC2->CC[0].bit.CC != 0)
    {
        newPulseLenth = TCC2->CC[0].bit.CC;
        TCC2->CC[0].bit.CC = 0;

        float pos = (newPulseLenth - 1500 * 24) * scale + offset;

        float currentPos = DCServoCommunicationHandler::dcServo->getPosition();
        pos = currentPos + std::max(std::min(pos - currentPos, 1000.0f), -1000.0f);

        if (state == 10)
        {
            float refPos = pos;

            DCServoCommunicationHandler::dcServo->loadNewReference(refPos, 0, 0);
            DCServoCommunicationHandler::dcServo->openLoopMode(false);
            DCServoCommunicationHandler::dcServo->enable(true);

            DCServoCommunicationHandler::statusLight.showEnabled();
        }
        else
        {
            ++state;
        }
    }
    else if (static_cast<int32_t>(time - lastPulseTime) > 100000)
    {
        lastPulseTime = time;

        state = 0;
        DCServoCommunicationHandler::onComIdleEvent();
    }
}

void DCServoCommunicationHandlerWithPwmInterface::config_tcc()
{
    // Init power and clock buses
    PM->APBCMASK.reg |= PM_APBCMASK_TCC2;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TCC2_GCLK_ID) |
                        GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN(0);
    while(GCLK->STATUS.bit.SYNCBUSY);

    // Reset TCCx
    TCC2->CTRLA.reg = TCC_CTRLA_SWRST;
    while(TCC2->SYNCBUSY.bit.SWRST);

    // Enable capture on all channels (just because)
    TCC2->CTRLA.reg  |= TCC_CTRLA_CPTEN0 | TCC_CTRLA_CPTEN1 | 
                            TCC_CTRLA_CPTEN2 | TCC_CTRLA_CPTEN3 |
                    TCC_CTRLA_PRESCALER_DIV2;
    // Enable event input (TCEIx), Match/Compare event input (MCEIx) and PWP mode
    TCC2->EVCTRL.reg |= TCC_EVCTRL_TCEI1 | 
                            TCC_EVCTRL_MCEI0 | TCC_EVCTRL_MCEI1 | 
                TCC_EVCTRL_EVACT1_PWP;
    while(TCC2->SYNCBUSY.reg);

    // Starting TCCx
    TCC2->CTRLA.bit.ENABLE = 1;
    while(TCC2->SYNCBUSY.reg);
}

/* Sense: 
 * None, Rise, Fall, Both, High, Low
 * 0x0   0x1   0x2   0x3   0x4   0x5
 */
void DCServoCommunicationHandlerWithPwmInterface::config_eic_channel(int ch, int sense, bool filt)
{
    // Config channel
    EIC->CONFIG[ch/8].reg &= ~(0xF << 4*(ch%8));
    EIC->CONFIG[ch/8].reg |= (0xF & ((filt? 0x8 : 0) | (0x7 & sense))) << 4*(ch%8);
    // No wake-up
    EIC->WAKEUP.reg &= ~(1 << ch);  
    // No interrupt
    EIC->INTENCLR.reg |= 1<<ch;
    // Generate Event 
    EIC->EVCTRL.reg |= 1<<ch;
}

void DCServoCommunicationHandlerWithPwmInterface::config_eic()
{
    PM->APBAMASK.reg |= PM_APBAMASK_EIC;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EIC_GCLK_ID) | 
                        GCLK_CLKCTRL_CLKEN | 
                        GCLK_CLKCTRL_GEN(0);
    EIC->CTRL.reg = EIC_CTRL_SWRST;
    while(EIC->CTRL.bit.SWRST && EIC->STATUS.bit.SYNCBUSY);
    config_eic_channel(11, 4, false);

    EIC->CTRL.bit.ENABLE = 1;
    while(EIC->STATUS.bit.SYNCBUSY);
}

void DCServoCommunicationHandlerWithPwmInterface::config_evsys()
{
    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EVSYS_GCLK_ID_0) |
                        GCLK_CLKCTRL_CLKEN |
                        GCLK_CLKCTRL_GEN(0);
    while(GCLK->STATUS.bit.SYNCBUSY);

    EVSYS->CTRL.bit.SWRST = 1;
    while(EVSYS->CTRL.bit.SWRST);

    // Event receiver
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) | // Set channel n-1
                  EVSYS_USER_USER(EVSYS_ID_USER_TCC2_EV_1); // Match/Capture 1 on TCC2
    // Event channel
    EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0) | // Set channel n
                 EVSYS_CHANNEL_PATH_ASYNCHRONOUS | 
                     EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11) |
                     EVSYS_CHANNEL_EDGSEL_BOTH_EDGES; // Detect both edges  
    // Wait channel to be ready
    while(!EVSYS->CHSTATUS.bit.USRRDY0);
    // EVSYS is always enabled
}

void DCServoCommunicationHandlerWithPwmInterface::gpio_in(int port, int pin)
{
    PORT->Group[port].DIRCLR.reg = (1 << pin);
    PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_INEN;
}

void DCServoCommunicationHandlerWithPwmInterface::gpio_pmuxen(int port, int pin, int mux)
{
    PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
    if (pin & 1)                                
        PORT->Group[port].PMUX[pin>>1].bit.PMUXO = mux;
    else
        PORT->Group[port].PMUX[pin>>1].bit.PMUXE = mux;
}

void DCServoCommunicationHandlerWithPwmInterface::config_gpio()
{
    gpio_in(0, 11);
    gpio_pmuxen(0, 11, PINMUX_PA11A_EIC_EXTINT11);
}

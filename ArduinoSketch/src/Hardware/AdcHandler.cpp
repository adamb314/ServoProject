#include "AdcHandler.h"

#include "wiring_private.h"
// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
}

AdcSamplerInstance::AdcSamplerInstance(uint32_t p) :
    pin(p)
{
    if (pin < A0) {
        pin += A0;
    }

    pinPeripheral(pin, PIO_ANALOG);

    // Disable DAC, if analogWrite() was used previously to enable the DAC
    if ((g_APinDescription[pin].ulADCChannelNumber == ADC_Channel0) || (g_APinDescription[pin].ulADCChannelNumber == DAC_Channel0)) {
        syncDAC();
        DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
        //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
        syncDAC();
    }

    // Control A
    /*
    * Bit 1 ENABLE: Enable
    *   0: The ADC is disabled.
    *   1: The ADC is enabled.
    * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
    * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
    * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
    *
    * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
    * configured. The first conversion after the reference is changed must not be used.
    */

    
    if (ADC->CTRLA.bit.ENABLE != 0x01)
    {
        syncADC();
        ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
    }
}

AdcSamplerInstance::~AdcSamplerInstance()
{
    unlockFromAdc();
}

void AdcSamplerInstance::getAdcLockAndStartSampling()
{
    bool startSampling = false;

    noInterrupts();
    if (!pendingInQueue)
    {
        pendingInQueue = true;
        if (AdcHandler::endOfQueueInstance == nullptr)
        {
            AdcHandler::endOfQueueInstance = this;
            AdcHandler::activeInstance = this;
            startSampling = true;         
        }
        else
        {
            auto currentEnd = AdcHandler::endOfQueueInstance;
            currentEnd->nextQueued = this;
            this->preQueued = currentEnd;
            AdcHandler::endOfQueueInstance = this;
        }
    }
    interrupts();

    if (startSampling)
    {
        loadConfigAndStart();
    }
}

void AdcSamplerInstance::unlockFromAdc()
{
    while (pendingInQueue)
    {
    }
}

bool AdcSamplerInstance::sampleReady()
{
    return !pendingInQueue;
}

void AdcSamplerInstance::startAdcSample()
{
    syncADC();
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input

    syncADC();
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
    ADC->SWTRIG.bit.START = 1;
}

AdcSamplerInstance* AdcHandler::activeInstance = nullptr;
AdcSamplerInstance* AdcHandler::endOfQueueInstance = nullptr;

void AdcHandler::init()
{
    analogReference(AR_DEFAULT);
    analogReadResolution(12);

    //ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_1_Val;
    //ADC->AVGCTRL.bit.ADJRES = 0x0ul;
    //ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
    //ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    //ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(5);

    ADC->INTFLAG.bit.RESRDY = 1;
    ADC->INTENSET.bit.RESRDY = 1;
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_Handler()
{
    noInterrupts();
    auto currentActive = AdcHandler::activeInstance;
    if (currentActive != nullptr)
    {
        if (currentActive->handleResultAndCleanUp(ADC->RESULT.reg))
        {
            auto nextActive = currentActive->nextQueued;
            if (nextActive != nullptr)
            {
                nextActive->preQueued = nullptr;
                currentActive->nextQueued = nullptr;
                AdcHandler::activeInstance = nextActive;
                currentActive->pendingInQueue = false;

                interrupts();
                AdcHandler::activeInstance->loadConfigAndStart();
                ADC->INTFLAG.bit.RESRDY = 1;
                return;
            }
            else
            {
                AdcHandler::activeInstance = nullptr;
                AdcHandler::endOfQueueInstance = nullptr;
                currentActive->pendingInQueue = false;
            }
            currentActive->handleRetriggering();
        }
        else
        {
            interrupts();
            ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
            ADC->SWTRIG.bit.START = 1;
            return;
        }
    }
    interrupts();
    ADC->INTFLAG.bit.RESRDY = 1;
}

AnalogSampler::AnalogSampler(uint32_t pin, uint8_t prescalerDivEnum) :
    AdcSamplerInstance(pin),
    prescalerDivEnum(prescalerDivEnum)
{
}

AnalogSampler::~AnalogSampler()
{
}

void AnalogSampler::triggerSample(uint8_t sampleNrEnum)
{
    this->sampleNrEnum = sampleNrEnum;
    autoRetrigger = false;
    AdcSamplerInstance::getAdcLockAndStartSampling();
}

void AnalogSampler::startAutoSampling(uint8_t sampleNrEnum)
{
    this->sampleNrEnum = sampleNrEnum;
    autoRetrigger = true;
    AdcSamplerInstance::getAdcLockAndStartSampling();
}

int32_t AnalogSampler::getValue()
{
    if (autoRetrigger)
    {
        noInterrupts();
        int32_t out = value;
        interrupts();
        return out;
    }

    AdcSamplerInstance::unlockFromAdc();
    return value;
}

void AnalogSampler::loadConfigAndStart()
{
    defaultSAMPLENUM = ADC->AVGCTRL.bit.SAMPLENUM;
    defaultRESSEL = ADC->CTRLB.bit.RESSEL;
    defaultPRESCALER = ADC->CTRLB.bit.PRESCALER;

    ADC->AVGCTRL.bit.SAMPLENUM = sampleNrEnum;
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
    ADC->CTRLB.bit.PRESCALER = prescalerDivEnum;

    AdcSamplerInstance::startAdcSample();
}

bool AnalogSampler::handleResultAndCleanUp(int32_t result)
{
    ADC->AVGCTRL.bit.SAMPLENUM = defaultSAMPLENUM;
    ADC->CTRLB.bit.RESSEL = defaultRESSEL;
    ADC->CTRLB.bit.PRESCALER = defaultPRESCALER;

    value = result;
    return true;
}

void AnalogSampler::handleRetriggering()
{
    if (autoRetrigger)
    {
        AdcSamplerInstance::getAdcLockAndStartSampling();
    }
}

AverageAnalogSampler::AverageAnalogSampler(uint32_t pin) :
    AdcSamplerInstance(pin)
{
}

AverageAnalogSampler::~AverageAnalogSampler()
{
}

void AverageAnalogSampler::triggerSample()
{
    AdcSamplerInstance::getAdcLockAndStartSampling();
}

int32_t AverageAnalogSampler::getValue()
{
    AdcSamplerInstance::unlockFromAdc();
    return value;
}

void AverageAnalogSampler::loadConfigAndStart()
{
    defaultSAMPLENUM = ADC->AVGCTRL.bit.SAMPLENUM;
    defaultPRESCALER = ADC->CTRLB.bit.PRESCALER;
    defaultRESSEL = ADC->CTRLB.bit.RESSEL;
    defaultSAMPLEN = ADC->SAMPCTRL.bit.SAMPLEN;

    ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_64_Val;
    ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV8_Val;
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
    ADC->SAMPCTRL.bit.SAMPLEN = 8;

    AdcSamplerInstance::startAdcSample();
}

bool AverageAnalogSampler::handleResultAndCleanUp(int32_t result)
{
    ADC->AVGCTRL.bit.SAMPLENUM = defaultSAMPLENUM;
    ADC->CTRLB.bit.PRESCALER = defaultPRESCALER;
    ADC->CTRLB.bit.RESSEL = defaultRESSEL;
    ADC->SAMPCTRL.bit.SAMPLEN = defaultSAMPLEN;

    value = result;

    return true;
}

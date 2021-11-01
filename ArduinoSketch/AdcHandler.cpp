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
    noInterrupts();
    if (AdcHandler::activeInstance != nullptr)
    {
        AdcHandler::activeInstance->unlockFromAdc(this);
    }
    else
    {
        AdcHandler::activeInstance = this;
    }
    interrupts();

    loadConfig();

    syncADC();
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
    ADC->SWTRIG.bit.START = 1;
}

void AdcSamplerInstance::unlockFromAdc(AdcSamplerInstance* newInstance)
{
    noInterrupts();
    if (AdcHandler::activeInstance == this)
    {
        interrupts();
        while (!ADC->INTFLAG.bit.RESRDY)
        {
        }

        unloadConfig(ADC->RESULT.reg);

        noInterrupts();
        AdcHandler::activeInstance = newInstance;
    }
    interrupts();
}

bool AdcSamplerInstance::sampleReady()
{
    noInterrupts();
    AdcSamplerInstance* tempActiveInst = AdcHandler::activeInstance;
    bool tempAdcReady = ADC->INTFLAG.bit.RESRDY;
    interrupts();

    if (tempActiveInst == this)
    {
        if (!tempAdcReady)
        {
            return false;
        }
    }

    return true;
}

void AdcSamplerInstance::configureAdcPin()
{
    syncADC();
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
}

AdcSamplerInstance* AdcHandler::activeInstance = nullptr;

void AdcHandler::init()
{
    analogReference(AR_DEFAULT);
    analogReadResolution(12);

    //ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_1_Val;
    //ADC->AVGCTRL.bit.ADJRES = 0x0ul;
    //ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
    //ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    //ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(5);
}

AnalogSampler::AnalogSampler(uint32_t pin) :
    AdcSamplerInstance(pin),
    value(0)
{
}

AnalogSampler::~AnalogSampler()
{
}

void AnalogSampler::triggerSample()
{
    AdcSamplerInstance::getAdcLockAndStartSampling();
}

int32_t AnalogSampler::getValue()
{
    AdcSamplerInstance::unlockFromAdc();
    return value;
}

void AnalogSampler::loadConfig()
{
    AdcSamplerInstance::configureAdcPin();
}

void AnalogSampler::unloadConfig(int32_t result)
{
    value = result;
}

AverageAnalogSampler::AverageAnalogSampler(uint32_t pin) :
    AnalogSampler(pin)
{
}

AverageAnalogSampler::~AverageAnalogSampler()
{
}

void AverageAnalogSampler::loadConfig()
{
    defaultSAMPLENUM = ADC->AVGCTRL.bit.SAMPLENUM;
    defaultPRESCALER = ADC->CTRLB.bit.PRESCALER;
    defaultRESSEL = ADC->CTRLB.bit.RESSEL;
    defaultSAMPLEN = ADC->SAMPCTRL.bit.SAMPLEN;

    ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_64_Val;
    ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV8_Val;
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
    ADC->SAMPCTRL.bit.SAMPLEN = 8;

    AnalogSampler::loadConfig();
}

void AverageAnalogSampler::unloadConfig(int32_t result)
{
    AnalogSampler::unloadConfig(result);

    ADC->AVGCTRL.bit.SAMPLENUM = defaultSAMPLENUM;
    ADC->CTRLB.bit.PRESCALER = defaultPRESCALER;
    ADC->CTRLB.bit.RESSEL = defaultRESSEL;
    ADC->SAMPCTRL.bit.SAMPLEN = defaultSAMPLEN;
}

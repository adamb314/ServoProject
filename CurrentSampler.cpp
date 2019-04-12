#include "CurrentSampler.h"

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

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

CurrentSampler::CurrentSampler() :
    value(0),
    offset(0),
    activeSample(false)
{
    analogReference(AR_DEFAULT);
    analogReadResolution(16);

    syncDAC();
    ADC->AVGCTRL.bit.SAMPLENUM = ADC_AVGCTRL_SAMPLENUM_64_Val;
    ADC->AVGCTRL.bit.ADJRES = 0x0ul;
    ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV8_Val;
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;
    ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(8);
    syncDAC();
}

CurrentSampler::~CurrentSampler()
{}

void CurrentSampler::init(uint32_t pin)
{
    configureAdcPin(pin);

    const size_t numberOfReads = 100;
    offset = 0;
    int32_t temp = 0;
    for (size_t i = 0; i < numberOfReads; i++)
    {
        triggerSample();
        temp += getValue();
    }

    offset = temp / numberOfReads;

    value = 0;
}


void CurrentSampler::triggerSample()
{
    ADC->SWTRIG.bit.START = 1;

    activeSample = true;
}

void CurrentSampler::resetFilteredValue()
{
    filteredValue = value;
}

int32_t CurrentSampler::getValue()
{
    collectSample();

    return value;
}

int32_t CurrentSampler::getFilteredValue()
{
    collectSample();

    return (filteredValue >> 3);
}

void CurrentSampler::configureAdcPin(uint32_t pin)
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

    syncADC();
    ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input

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
    syncADC();
    ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

    // Start conversion
    syncADC();
    ADC->SWTRIG.bit.START = 1;

    // Clear the Data Ready flag
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

    // Start conversion again, since The first conversion after the reference is changed must not be used.
    syncADC();

}

void CurrentSampler::collectSample()
{
    if (!activeSample)
    {
        return;
    }

    // Store the value
    while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete

    value = ADC->RESULT.reg - offset;

    filteredValue = ((7 * filteredValue) >> 3) + value;
}

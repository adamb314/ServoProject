#ifndef ADC_HANDLER_H
#define ADC_HANDLER_H

#include <Arduino.h>
#undef max
#undef min

void ADC_Handler();

class AdcSamplerInstance
{
public:
    AdcSamplerInstance(uint32_t pin);
    virtual ~AdcSamplerInstance();

    bool sampleReady();

protected:
    void getAdcLockAndStartSampling();

    void unlockFromAdc();

    void startAdcSample();

    virtual void loadConfigAndStart() = 0;
    virtual bool handleResultAndCleanUp(int32_t result) = 0;
    virtual void handleRetriggering() {};

private:
    uint32_t pin;

    AdcSamplerInstance* preQueued{nullptr};
    AdcSamplerInstance* nextQueued{nullptr};
    volatile bool pendingInQueue{false};

    friend void ADC_Handler();
};

class AdcHandler
{
public:
    static void init();

private:
    static AdcSamplerInstance* activeInstance;
    static AdcSamplerInstance* endOfQueueInstance;

    friend class AdcSamplerInstance;
    friend void ADC_Handler();
};

class AnalogSampler : public AdcSamplerInstance
{
public:
    AnalogSampler(uint32_t pin, uint8_t prescalerDivEnum = ADC_CTRLB_PRESCALER_DIV32_Val);

    virtual ~AnalogSampler();

    void triggerSample(uint8_t sampleNrEnum = ADC_AVGCTRL_SAMPLENUM_1_Val);

    void startAutoSampling(uint8_t sampleNrEnum = ADC_AVGCTRL_SAMPLENUM_1_Val);

    int32_t getValue();

protected:
    virtual void loadConfigAndStart() override;

    virtual bool handleResultAndCleanUp(int32_t result) override;

    virtual void handleRetriggering() override;

    int32_t value{0};

private:
    uint8_t sampleNrEnum;
    bool autoRetrigger{false};
    uint8_t prescalerDivEnum;
    uint8_t defaultSAMPLENUM;
    uint16_t defaultRESSEL;
    uint8_t defaultPRESCALER;
};

class AverageAnalogSampler : public AdcSamplerInstance
{
public:
    AverageAnalogSampler(uint32_t pin);
    virtual ~AverageAnalogSampler();

    void triggerSample();

    int32_t getValue();

protected:
    virtual void loadConfigAndStart() override;
    virtual bool handleResultAndCleanUp(int32_t result) override;

    int32_t value{0};

private:
    uint8_t defaultSAMPLENUM;
    uint16_t defaultPRESCALER;
    uint16_t defaultRESSEL;
    uint8_t defaultSAMPLEN;
};

#endif

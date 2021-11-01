#ifndef ADC_HANDLER_H
#define ADC_HANDLER_H

#include <Arduino.h>
#undef max
#undef min

class AdcSamplerInstance
{
public:
    AdcSamplerInstance(uint32_t pin);
    virtual ~AdcSamplerInstance();

    bool sampleReady();

protected:
    void getAdcLockAndStartSampling();

    void unlockFromAdc(AdcSamplerInstance* newInstance = nullptr);


    void configureAdcPin();

    virtual void loadConfig() = 0;
    virtual void unloadConfig(int32_t result) = 0;

private:
    uint32_t pin;
};

class AdcHandler
{
public:
    static void init();

private:
    static AdcSamplerInstance* activeInstance;

    friend class AdcSamplerInstance;
};

class AnalogSampler : public AdcSamplerInstance
{
public:
    AnalogSampler(uint32_t pin);

    virtual ~AnalogSampler();

    void triggerSample();

    int32_t getValue();

protected:
    virtual void loadConfig() override;

    virtual void unloadConfig(int32_t result) override;

private:
    int32_t value;
};

class AverageAnalogSampler : public AnalogSampler
{
public:
    AverageAnalogSampler(uint32_t pin);
    virtual ~AverageAnalogSampler();

protected:
    virtual void loadConfig() override;
    virtual void unloadConfig(int32_t result) override;

private:
    uint8_t defaultSAMPLENUM;
    uint16_t defaultPRESCALER;
    uint16_t defaultRESSEL;
    uint8_t defaultSAMPLEN;
};

#endif

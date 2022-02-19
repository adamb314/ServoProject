extern uint32_t getInterruptTimerTick();

#if defined(_SAMD21_)
extern "C"
{
void tc5InterruptRunCaller();
}

class InterruptTimer : public ThreadHandler::InterruptTimerInterface
{
public:
    static InterruptTimer* getInstance();

    virtual ~InterruptTimer(){};

    virtual void enableNewInterrupt() override;

    virtual void blockInterrupts() override;
    virtual void unblockInterrupts() override;

    virtual uint32_t syncedMicros() override;

    virtual uint32_t getInterruptTimestamp() override;

    static  void enableNewInterruptImp();

    static  void blockInterruptsImp();
    static  void unblockInterruptsImp();

    static  uint32_t syncedMicrosImp();
 
    static  uint32_t getInterruptTimestampImp();

private:
    InterruptTimer(uint16_t interruptTick);

    static void interruptRun();

    static void configure(uint16_t period);

    static bool isSyncing();

    static void startCounter();

    static void reset();

    static void disable();

    static uint16_t interruptTick;

    static uint32_t interruptTimerTime;

    static uint32_t microsTimerOffset;

    friend void tc5InterruptRunCaller();
};
#elif defined(__AVR__)
#include <TimerOne.h>

void interruptHandler();

class InterruptTimer : public ThreadHandler::InterruptTimerInterface
{
public:
    static InterruptTimer* getInstance();

    virtual ~InterruptTimer(){};

    virtual void enableNewInterrupt() override;

    virtual void blockInterrupts() override;
    virtual void unblockInterrupts() override;

    virtual uint32_t syncedMicros() override;

    virtual uint32_t getInterruptTimestamp() override;

    static  void enableNewInterruptImp();

    static  void blockInterruptsImp();
    static  void unblockInterruptsImp();

    static  uint32_t syncedMicrosImp();
 
    static  uint32_t getInterruptTimestampImp();

private:
    InterruptTimer(uint16_t interruptTick);

    static void interruptRun();

    static uint32_t microsAtInterrupt;

    friend void interruptHandler();
};
#endif

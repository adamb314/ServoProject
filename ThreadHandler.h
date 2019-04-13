#ifndef THREAD_HANDLER_H
#define THREAD_HANDLER_H

#ifdef NOT_ARDUINO
    #include <cinttypes>
    void simulateMicrosTimeStep();
    uint32_t micros();
    const int8_t TC5_IRQn = 0;
    void NVIC_DisableIRQ(int8_t i);
    void NVIC_EnableIRQ(int8_t i);
#else
#include <Arduino.h>
#undef min
#undef max
#include <memory>
#include "ArduinoC++BugFixes.h"
#endif

#include <vector>
#include <algorithm>
#include <stdint.h>
#include <functional>

extern void createAndConfigureThreadHandler();
#define START_OF_THREAD_HANDLER_CONFIG \
ThreadHandler* globalThreadHandlerInst = nullptr; void createAndConfigureThreadHandler()

#define THREAD_HANDLER(interruptTick) \
{ \
    static ThreadHandler threadHandler(interruptTick); \
    globalThreadHandlerInst = &threadHandler; \


#define THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(interruptTick) \
{ \
    static ThreadHandlerExecutionOrderOptimized threadHandler(interruptTick); \
    globalThreadHandlerInst = &threadHandler; \

#define END_OF_THREAD_HANDLER_CONFIG }

class Thread
{
public:
    Thread(int8_t priority, uint32_t period, uint32_t startOffset);

    virtual ~Thread();

    virtual void run() = 0;
};

class FunctionThread : public Thread
{
public:
    FunctionThread(int8_t priority, uint32_t period, uint32_t startOffset, std::function<void(void)> fun) :
        Thread(priority, period, startOffset),
        fun(fun)
    {
    }

    virtual ~FunctionThread()
    {
    }

    virtual void run()
    {
        fun();
    }

private:
    std::function<void(void)> fun;
};

class ThreadInterruptBlocker
{
public:
    ThreadInterruptBlocker();

    ~ThreadInterruptBlocker();

    void lock();

    void unlock();

private:
    static uint32_t blockerCount;

    bool iAmLocked;
};

class ThreadHandler
{
public:
    static ThreadHandler* getInstance();

    virtual ~ThreadHandler();

    void run();

    uint16_t getCpuLoad();

protected:
    ThreadHandler(uint16_t interruptTick);

    virtual void add(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t);

    virtual void remove(const Thread* t);

    class InternalThreadHolder
    {
    public:
        InternalThreadHolder(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t);

        ~InternalThreadHolder();

        void updateCurrentTime(uint32_t time);

        bool pendingRun();

        bool higherPriorityThan(const InternalThreadHolder* other);
        bool higherPriorityThan(const InternalThreadHolder& other);

        void runThread();

        bool isHolderFor(const Thread* t) const;

        int8_t getPriority() const;

        static std::vector<InternalThreadHolder*> generateExecutionOrderVector(const std::vector<InternalThreadHolder*>& threadHolders);

    private:
        Thread* thread;

        bool initiated;
        uint32_t runAtTimestamp;
        int32_t timeUntillRun;
        uint32_t period;
        uint32_t startOffset;
        int8_t priority;
    };

    void generateExecutionOrder();

    virtual InternalThreadHolder* getNextThreadToRun(uint32_t currentTimestamp);

#ifndef NOT_ARDUINO
    void interruptRun();

    class InterruptTimer
    {
    public:
        InterruptTimer(uint16_t interruptTick);

        ~InterruptTimer();

        void enableNewInterrupt();
        
    private:
        void configure(uint16_t period);

        bool isSyncing();

        void startCounter();

        void reset();

        void disable();
    };
    InterruptTimer* interruptTimer;

    friend void TC5_Handler();
#endif

    int8_t priorityOfRunningThread;
    uint32_t cpuLoadTime;
    uint32_t totalTime;
    std::vector<InternalThreadHolder> threadHolders;

    friend Thread;
    friend void createAndConfigureThreadHandler();
};

class ThreadHandlerExecutionOrderOptimized : public ThreadHandler
{
public:
    virtual ~ThreadHandlerExecutionOrderOptimized();

    virtual void add(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t);

    virtual void remove(const Thread* t);

private:
    ThreadHandlerExecutionOrderOptimized(uint16_t interruptTick);

    class ThreadHolderPriorityGroup
    {
    public:
        ThreadHolderPriorityGroup(InternalThreadHolder* t);

        void add(InternalThreadHolder* t);

        void remove(const InternalThreadHolder* th);

        int8_t getPriority();

        void generateExecutionOrder();

        InternalThreadHolder* getNextThreadToRun(uint32_t currentTimestamp);

    private:
        int8_t priority;
        std::vector<InternalThreadHolder*> threadHolders;
        std::vector<InternalThreadHolder*> executeRingBuffer;
        std::vector<InternalThreadHolder*>::iterator itToNextThreadToRun;
    };

    void generateExecutionOrder();

    virtual InternalThreadHolder* getNextThreadToRun(uint32_t currentTimestamp);

    bool executionOrderGenerated;
    std::vector<ThreadHolderPriorityGroup> priorityGroups;

    friend void createAndConfigureThreadHandler();
};

extern ThreadHandler* globalThreadHandlerInst;

#endif

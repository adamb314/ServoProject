#ifndef THREAD_HANDLER_H
#define THREAD_HANDLER_H

#include <Arduino.h>

#include "configuringMacros.h"

template <typename returnType = void>
class FunctionalWrapper
{
public:
    FunctionalWrapper(){};
    ~FunctionalWrapper(){};

    virtual returnType operator()() = 0;
};

template <typename F, typename returnType = void>
class FunctionalWrapperTemplate : public FunctionalWrapper<returnType>
{
public:
    FunctionalWrapperTemplate(F fun) :
        fun(fun)
    {
    }

    ~FunctionalWrapperTemplate(){};

    virtual returnType operator()()
    {
        return fun();
    }

private:
    F fun;
};

template <typename returnType = void, typename F>
FunctionalWrapper<returnType>* createFunctionalWrapper(F fun)
{
    return new FunctionalWrapperTemplate<F, returnType>(fun);
}

class CodeBlocksThread;
class ThreadHandler;

class Thread
{
public:
    Thread(int8_t priority, int32_t period, uint32_t startOffset = 0);

    virtual ~Thread();

    virtual void run() = 0;

    static void delayNextCodeBlock(int32_t delay);

    static void delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun);

    void disableExecution();

    void enableExecution(int32_t period = 0, uint32_t startOffset = 0);

    static uint32_t getTimingError();

    int8_t getPriority() const;

    bool inRunQueue() const;

private:
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    virtual bool splitIntoCodeBlocks();

    virtual void internalDelayNextCodeBlock(int32_t delay);
    virtual bool firstCodeBlock();


    virtual void internalDelayNextCodeBlockUntil(FunctionalWrapper<bool>* fun);

    uint32_t internalGetTimingError();

    void initiate(uint32_t currnetTime);

    virtual void updateCurrentTime(uint32_t time);

    bool pendingRun();

    bool higherPriorityThan(const Thread* other);
    bool higherPriorityThan(const Thread& other);

    void runThread();

private:
    bool initiated{false};
    int8_t priority;
    uint32_t runAtTimestamp{0};
    int32_t timeUntillRun{0};
    int32_t period;
    uint32_t startOffset{0};

    Thread* previous{nullptr};
    Thread* next{nullptr};
    Thread* nextPendingRun{this};

    friend class ThreadHandler;
    friend class CodeBlocksThread;
};

template <typename F>
class FunctionThreadTemplate : public Thread
{
public:
    FunctionThreadTemplate(int8_t priority, int32_t period, uint32_t startOffset, F fun) :
        Thread(priority, period, startOffset),
        fun(fun)
    {
    }

    virtual ~FunctionThreadTemplate()
    {
    }

    virtual void run()
    {
        fun();
    }

private:
    F fun;
};

template <typename F>
FunctionThreadTemplate<F>* createThread(int8_t priority, int32_t period, uint32_t startOffset, F fun)
{
    return new FunctionThreadTemplate<F>(priority, period, startOffset, fun);
}

class CodeBlocksThread : public Thread
{
public:
    template <typename F>
    CodeBlocksThread(int8_t priority, int32_t period, uint32_t startOffset, F fun);

    virtual ~CodeBlocksThread();

    template <typename F>
    void addCodeBlock(F fun);

private:
    virtual void run() override;

    virtual void updateCurrentTime(uint32_t currnetTime) override;

    virtual bool firstCodeBlock() override;

    virtual bool splitIntoCodeBlocks() override;

    virtual void internalDelayNextCodeBlock(int32_t delay) override;

    virtual void internalDelayNextCodeBlockUntil(FunctionalWrapper<bool>* fun) override;

    class Node
    {
    public:
        Node(FunctionalWrapper<>* fun) : fun(fun) {};

        Node* next{nullptr};

        FunctionalWrapper<>* fun{nullptr};
    };

    Node* funListStart{nullptr};
    Node* funListLast{nullptr};
    Node* nextFunBlockNode{nullptr};
    FunctionalWrapper<bool>* delayCodeBlockUntilFun{nullptr};
};

template <typename F>
CodeBlocksThread* createThreadWithCodeBlocks(int8_t priority, int32_t period, uint32_t startOffset, F fun)
{
    return new CodeBlocksThread(priority, period, startOffset, fun);
}

class ThreadInterruptBlocker
{
public:
    ThreadInterruptBlocker();

    ~ThreadInterruptBlocker();

    void lock();

    void unlock();

    static unsigned int blockerCount;
    bool iAmLocked{false};
private:

};

class ThreadHandler
{
public:
    static ThreadHandler* getInstance();

    virtual ~ThreadHandler();

    void enableThreadExecution(bool enable = true);

    uint8_t getCpuLoad();

    void delayNextCodeBlock(int32_t delay);

    void delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun);

    uint32_t getTimingError();

    int8_t getExecutionHaltedOnPriorityAfterDelete();

    class InterruptTimerInterface
    {
    public:
        virtual ~InterruptTimerInterface(){};

        virtual void enableNewInterrupt(){};

        virtual void blockInterrupts(){};
        virtual void unblockInterrupts(){};

        virtual uint32_t syncedMicros(){return micros();};

        virtual uint32_t getInterruptTimestamp(){return micros();};

    protected:
        static void interruptRun();
    };

private:
    ThreadHandler(const ThreadHandler&) = delete;
    ThreadHandler& operator=(const ThreadHandler&) = delete;

    static InterruptTimerInterface* interruptTimer;

    template <typename T>
    static T getClassType(T* inst);

    template<typename T>
    static void callBlock();

    template<typename T>
    static void callUnblock();

    template<typename T>
    static void callEnableNewInterrupt();
    template<typename T>
    static uint32_t callSyncedMicros();

    template<typename T>
    static uint32_t callGetInterruptTimestamp();

protected:
    ThreadHandler();

    virtual void add(Thread* t);

    virtual void remove(const Thread* t);

    Thread* getNextThreadToRunAndRemoveFrom(Thread*& head);

    virtual Thread* getHeadOfThreadsToRun(uint32_t currentTimestamp);

    void interruptRun();

    static void blockInterrupts();
    static void unblockInterrupts();
    static void enableNewInterrupt();
    static uint32_t syncedMicros();
    static uint32_t getInterruptTimestamp();

    bool threadExecutionEnabled{false};
    int8_t executionHaltedOnPrio{-128};
    Thread* currentThread{nullptr};
    int8_t priorityOfRunningThread{-128};
    unsigned int cpuLoadTime{0};
    unsigned int totalTime{1};
    Thread* firstThread{nullptr};

    friend Thread;
    friend CodeBlocksThread;
    friend ThreadInterruptBlocker;
    friend ThreadHandler* createAndConfigureThreadHandler();
};

template <typename F>
CodeBlocksThread::CodeBlocksThread(int8_t priority, int32_t period, uint32_t startOffset, F fun) :
    Thread(priority, period, startOffset)
{
    addCodeBlock<F>(fun);
    nextFunBlockNode = funListStart;
}

template <typename F>
void CodeBlocksThread::addCodeBlock(F fun)
{
    Node* newNode = new Node(new FunctionalWrapperTemplate<F>(fun));

    if (funListLast != nullptr)
    {
        funListLast->next = newNode;
        funListLast = newNode;
    }
    else
    {
        funListStart = newNode;
        funListLast = newNode;
    }
}

#include "platformSpecificClasses.h"

#endif

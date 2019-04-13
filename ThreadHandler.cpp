#include "ThreadHandler.h"

ThreadHandler* threadHandlerInst = ThreadHandler::getInstance();

Thread::Thread(int8_t priority, uint32_t period, uint32_t startOffset)
{
    threadHandlerInst->add(priority, period, startOffset, this);
}

Thread::~Thread()
{
    threadHandlerInst->remove(this);
}

uint32_t ThreadInterruptBlocker::blockerCount = 0;

ThreadInterruptBlocker::ThreadInterruptBlocker() :
    iAmLocked(false)
{
    lock();
}

ThreadInterruptBlocker::~ThreadInterruptBlocker()
{
    unlock();
}

void ThreadInterruptBlocker::lock()
{
    NVIC_DisableIRQ(TC5_IRQn);
    if (!iAmLocked)
    {
        iAmLocked = true;
        blockerCount++;
    }
}

void ThreadInterruptBlocker::unlock()
{
    if (iAmLocked)
    {
        iAmLocked = false;
        blockerCount--;
    }
    if (blockerCount == 0)
    {
        NVIC_EnableIRQ(TC5_IRQn);
    }
}

ThreadHandler::InternalThreadHolder::InternalThreadHolder(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t) :
    thread(t),
    initiated(false),
    runAtTimestamp(0),
    timeUntillRun(0),
    period(period),
    priority(priority),
    startOffset(startOffset)
{
}

ThreadHandler::InternalThreadHolder::~InternalThreadHolder()
{
}

void ThreadHandler::InternalThreadHolder::updateCurrentTime(uint32_t currnetTime)
{
    if (!initiated)
    {
        runAtTimestamp = currnetTime + startOffset;
        initiated = true;
    }

    timeUntillRun = static_cast<int32_t>(runAtTimestamp - currnetTime);

    if (timeUntillRun < -(1 << 30))
    {
        runAtTimestamp += static_cast<uint32_t>(1 << 29);
    }
}

bool ThreadHandler::InternalThreadHolder::pendingRun()
{
    return timeUntillRun <= 0;
}

bool ThreadHandler::InternalThreadHolder::higherPriorityThan(const InternalThreadHolder* other)
{
    if (other == nullptr)
    {
        return true;
    }

    return higherPriorityThan(*other);
}

bool ThreadHandler::InternalThreadHolder::higherPriorityThan(const InternalThreadHolder& other)
{
    if (other.priority < priority)
    {
        return true;
    }
    else if (other.priority == priority)
    {
        if (other.timeUntillRun > timeUntillRun)
        {
            return true;
        }
    }

    return false;
}

void ThreadHandler::InternalThreadHolder::runThread()
{
    thread->run();
    runAtTimestamp += period;
}

bool ThreadHandler::InternalThreadHolder::isHolderFor(const Thread* t) const
{
    return thread == t;
}

int8_t ThreadHandler::InternalThreadHolder::getPriority() const
{
    return priority;
}

std::vector<ThreadHandler::InternalThreadHolder*> ThreadHandler::InternalThreadHolder::generateExecutionOrderVector(const std::vector<InternalThreadHolder*>& threadHolders)
{
    std::vector<InternalThreadHolder*> executeRingBuffer;

    if (threadHolders.size() == 1)
    {
        executeRingBuffer.push_back(threadHolders[0]);
    }
    else
    {
        std::vector<uint32_t> executeTimeRingBuffer;

        for (auto& th : threadHolders)
        {
            th->runAtTimestamp = th->startOffset;
        }

        while (true)
        {
            InternalThreadHolder* nextToExecute = threadHolders[0];
            for (auto& th : threadHolders)
            {
                if (nextToExecute->runAtTimestamp > th->runAtTimestamp)
                {
                    nextToExecute = th;
                }
            }

            executeRingBuffer.push_back(nextToExecute);
            executeTimeRingBuffer.push_back(nextToExecute->runAtTimestamp);
            nextToExecute->runAtTimestamp += nextToExecute->period;

            auto startPatternIterator = executeRingBuffer.begin();
            auto temp = std::find(executeRingBuffer.rbegin(), executeRingBuffer.rend(), executeRingBuffer.front());
            auto endPatternIterator = executeRingBuffer.begin() + std::distance(temp, executeRingBuffer.rend() - 1);

            size_t startPatternIndex = startPatternIterator - executeRingBuffer.begin();
            size_t endPatternIndex = endPatternIterator - executeRingBuffer.begin();

            if (startPatternIndex == endPatternIndex)
            {
                continue;
            }

            uint32_t startPatternTimeOffset = executeTimeRingBuffer[startPatternIndex];
            uint32_t endPatternTimeOffset = executeTimeRingBuffer[endPatternIndex];

            bool patternMach = true;
            for (auto& th : threadHolders)
            {
                auto itInStartPattern = std::find(startPatternIterator, executeRingBuffer.end(), th);
                auto itInEndPattern = std::find(endPatternIterator, executeRingBuffer.end(), th);

                if (itInStartPattern == executeRingBuffer.end() ||
                    itInEndPattern == executeRingBuffer.end())
                {
                    patternMach = false;
                    break;
                }

                size_t indexInStartPattern = itInStartPattern - startPatternIterator;
                size_t indexInEndPattern = itInEndPattern - startPatternIterator;

                if (executeTimeRingBuffer[indexInStartPattern] - startPatternTimeOffset !=
                    executeTimeRingBuffer[indexInEndPattern] - endPatternTimeOffset)
                {
                    patternMach = false;
                    break;
                }
            }

            if (patternMach)
            {
                executeRingBuffer.erase(endPatternIterator, executeRingBuffer.end());
                break;
            }
        }
    }

    for (auto& th : threadHolders)
    {
        th->initiated = false;
    }

    return executeRingBuffer;
}

ThreadHandler* ThreadHandler::getInstance()
{
    createAndConfigureThreadHandler();
    return globalThreadHandlerInst;
}

ThreadHandler::ThreadHandler(uint16_t interruptTick) :
    priorityOfRunningThread(-128),
    cpuLoadTime(0),
    totalTime(0)
{
#ifndef NOT_ARDUINO
    interruptTimer = new InterruptTimer(interruptTick);
#endif
}

ThreadHandler::~ThreadHandler()
{
}

void ThreadHandler::add(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t)
{
    threadHolders.push_back(InternalThreadHolder(priority, period, startOffset, t));
}

void ThreadHandler::remove(const Thread* t)
{
    threadHolders.erase(std::remove_if(threadHolders.begin(), threadHolders.end(),
        [t](const InternalThreadHolder& holder)
        {
            return holder.isHolderFor(t);
        }), threadHolders.end());
}

ThreadHandler::InternalThreadHolder* ThreadHandler::getNextThreadToRun(uint32_t currentTimestamp)
{
    InternalThreadHolder* threadToRunHolder = nullptr;

    for (auto& th : threadHolders)
    {
        if (th.getPriority() <= priorityOfRunningThread)
        {
            continue;
        }

        th.updateCurrentTime(currentTimestamp);

        if (th.pendingRun())
        {
            if (th.higherPriorityThan(threadToRunHolder))
            {
                threadToRunHolder = &th;
            }
        }
    }

    return threadToRunHolder;
}

void ThreadHandler::run()
{
    static uint32_t startTime = micros();
    uint32_t endTime;
    uint32_t currentTimestamp = micros();

    bool threadExecuted = false;

    while (true)
    {
        InternalThreadHolder* threadToRunHolder = getNextThreadToRun(currentTimestamp);

        if (threadToRunHolder != nullptr)
        {
            threadExecuted = true;
            priorityOfRunningThread = threadToRunHolder->getPriority();

            threadToRunHolder->runThread();

            priorityOfRunningThread = -128;
        }
        else
        {
            endTime = micros();
            uint32_t timeDiff = static_cast<int32_t>(endTime - startTime);
            startTime = endTime;

            totalTime += timeDiff;
            if (threadExecuted)
            {
                cpuLoadTime += timeDiff;
            }

            if (totalTime > 1000)
            {
                totalTime = (totalTime >> 1);
                cpuLoadTime = (cpuLoadTime >> 1);
            }
            return;
        }
    }
}

uint16_t ThreadHandler::getCpuLoad()
{
    uint16_t out = static_cast<uint16_t>((cpuLoadTime * 1000) / totalTime);
    return out;
}


ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::ThreadHolderPriorityGroup(InternalThreadHolder* t) :
    priority(t->getPriority())
{
    threadHolders.push_back(t);
}

void ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::add(InternalThreadHolder* t)
{
    threadHolders.push_back(t);
}

void ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::remove(const InternalThreadHolder* th)
{
    threadHolders.erase(std::remove(threadHolders.begin(), threadHolders.end(), th),
        threadHolders.end());
}

int8_t ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::getPriority()
{
    return priority;
}

void ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::generateExecutionOrder()
{
    executeRingBuffer = InternalThreadHolder::generateExecutionOrderVector(threadHolders);

    itToNextThreadToRun = executeRingBuffer.begin();
}

ThreadHandler::InternalThreadHolder* ThreadHandlerExecutionOrderOptimized::ThreadHolderPriorityGroup::getNextThreadToRun(uint32_t currentTimestamp)
{
    auto th = (*itToNextThreadToRun);

    th->updateCurrentTime(currentTimestamp);
    if (th->pendingRun())
    {
        ++itToNextThreadToRun;
        if (itToNextThreadToRun == executeRingBuffer.end())
        {
            itToNextThreadToRun = executeRingBuffer.begin();
        }

        return th;
    }

    return nullptr;
}

ThreadHandlerExecutionOrderOptimized::ThreadHandlerExecutionOrderOptimized(uint16_t interruptTick) :
    ThreadHandler(interruptTick),
    executionOrderGenerated(false)
{
}


void ThreadHandlerExecutionOrderOptimized::add(int8_t priority, uint32_t period, uint32_t startOffset, Thread* t)
{
    executionOrderGenerated = false;
    ThreadHandler::add(priority, period, startOffset, t);
}

void ThreadHandlerExecutionOrderOptimized::remove(const Thread* t)
{
    executionOrderGenerated = false;
    ThreadHandler::remove(t);
}

ThreadHandlerExecutionOrderOptimized::~ThreadHandlerExecutionOrderOptimized()
{
}

void ThreadHandlerExecutionOrderOptimized::generateExecutionOrder()
{
    priorityGroups.clear();

    for (auto& th : threadHolders)
    {
        auto it = priorityGroups.begin();
        for (; it != priorityGroups.end(); ++it)
        {
            if (th.getPriority() == it->getPriority())
            {
                it->add(&th);
                break;
            }
            else if (th.getPriority() > it->getPriority())
            {
                priorityGroups.insert(it, ThreadHolderPriorityGroup(&th));
                break;
            }
        }
        if (it == priorityGroups.end())
        {
            priorityGroups.insert(it, ThreadHolderPriorityGroup(&th));
        }
    }

    for (auto& pg : priorityGroups)
    {
        pg.generateExecutionOrder();
    }
}

ThreadHandler::InternalThreadHolder* ThreadHandlerExecutionOrderOptimized::getNextThreadToRun(uint32_t currentTimestamp)
{
    if (!executionOrderGenerated)
    {
        generateExecutionOrder();
        executionOrderGenerated = true;
    }

    for (auto& pg : priorityGroups)
    {
        if (pg.getPriority() <= priorityOfRunningThread)
        {
            return nullptr;
        }

        InternalThreadHolder* th = pg.getNextThreadToRun(currentTimestamp);

        if (th != nullptr)
        {
            return th;
        }
    }

    return nullptr;
}

#ifndef NOT_ARDUINO
void ThreadHandler::interruptRun()
{
    if (priorityOfRunningThread == -128)
    {
        interruptTimer->enableNewInterrupt();
        return;
    }

    uint32_t currentTimestamp = micros();
    InternalThreadHolder* threadToRunHolder = getNextThreadToRun(currentTimestamp);

    if (threadToRunHolder != nullptr)
    {
        int8_t temp = priorityOfRunningThread;
        priorityOfRunningThread = threadToRunHolder->getPriority();
        interruptTimer->enableNewInterrupt();

        threadToRunHolder->runThread();

        priorityOfRunningThread = temp;
    }
    else
    {
        interruptTimer->enableNewInterrupt();
    }
}

ThreadHandler::InterruptTimer::InterruptTimer(uint16_t interruptTick)
{
    configure(interruptTick);
    startCounter();
}

ThreadHandler::InterruptTimer::~InterruptTimer()
{
}

void ThreadHandler::InterruptTimer::enableNewInterrupt()
{
    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
}

void ThreadHandler::InterruptTimer::configure(uint16_t period)
{
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    while (GCLK->STATUS.bit.SYNCBUSY);

    disable();

    // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
    // Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
    //set prescaler and enable TC5
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;
    //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
    TC5->COUNT16.CC[0].reg = static_cast<uint16_t>(period * (F_CPU / 1000000ul));
    while (isSyncing());

    // Interrupts 
    TC5->COUNT16.INTENSET.reg = TC_INTENSET_OVF;          // enable overfollow

    // Enable InterruptVector
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 3);
    NVIC_EnableIRQ(TC5_IRQn);

    while (isSyncing()); //wait until TC5 is done syncing
}

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool ThreadHandler::InterruptTimer::isSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void ThreadHandler::InterruptTimer::startCounter()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
    while (isSyncing()); //wait until snyc'd
}

//Reset TC5 
void ThreadHandler::InterruptTimer::reset()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (isSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void ThreadHandler::InterruptTimer::disable()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (isSyncing());
}

extern "C"
{
//this function gets called by the TC5 interrupt
void TC5_Handler();
}

void TC5_Handler() {
    threadHandlerInst->interruptRun();
}

#endif

#ifdef NOT_ARDUINO
    static uint32_t microsTime = 0;
    void simulateMicrosTimeStep()
    {
        microsTime++;
    }

    uint32_t micros()
    {
        return microsTime;
    }

    void NVIC_DisableIRQ(int8_t i)
    {
    }

    void NVIC_EnableIRQ(int8_t i)
    {
    }
#endif

#include "ThreadHandler.h"

extern ThreadHandler::InterruptTimerInterface* getInterruptTimerInstance();

ThreadHandler::InterruptTimerInterface* ThreadHandler::interruptTimer = nullptr;

ThreadHandler* createAndConfigureThreadHandler()
{
    static ThreadHandler threadHandler;
    return &threadHandler;
}

CodeBlocksThread::~CodeBlocksThread()
{
    for (Node* it = funListStart; it != nullptr;)
    {
        Node* temp = it;
        it = temp->next;

        delete temp->fun;
        delete temp;
    }
}

void CodeBlocksThread::run()
{
    FunctionalWrapper<>& f = *(nextFunBlockNode->fun);
    f();
    nextFunBlockNode = nextFunBlockNode->next;
    if (nextFunBlockNode == nullptr)
    {
        nextFunBlockNode = funListStart;
    }
}

bool CodeBlocksThread::firstCodeBlock()
{
    return nextFunBlockNode == funListStart;
}

bool CodeBlocksThread::splitIntoCodeBlocks()
{
    return funListStart != funListLast;
}

void CodeBlocksThread::internalDelayNextCodeBlock(int32_t delay)
{
    runAtTimestamp = ThreadHandler::syncedMicros() + delay;
}

void CodeBlocksThread::internalDelayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
    delayCodeBlockUntilFun = fun;
}

void CodeBlocksThread::updateCurrentTime(uint32_t currentTime)
{
    Thread::initiate(currentTime);

    if (delayCodeBlockUntilFun != nullptr &&
        splitIntoCodeBlocks())
    {
        runAtTimestamp = currentTime;
        if ((*delayCodeBlockUntilFun)())
        {
            delayCodeBlockUntilFun = nullptr;
            timeUntillRun = 0;
        }
        else
        {
            runAtTimestamp += 1;
            timeUntillRun = 1;
        }
        return;
    }

    Thread::updateCurrentTime(currentTime);
}

unsigned int ThreadInterruptBlocker::blockerCount = 0;

ThreadInterruptBlocker::ThreadInterruptBlocker()
{
    lock();
}

ThreadInterruptBlocker::~ThreadInterruptBlocker()
{
    unlock();
}

void ThreadInterruptBlocker::lock()
{
    if (ThreadHandler::interruptTimer == nullptr)
    {
        return;
    }

    ThreadHandler::blockInterrupts();
    if (!iAmLocked)
    {
        iAmLocked = true;
        blockerCount++;
    }
}

void ThreadInterruptBlocker::unlock()
{
    if (ThreadHandler::interruptTimer == nullptr)
    {
        return;
    }

    if (iAmLocked)
    {
        iAmLocked = false;
        blockerCount--;
    }
    if (blockerCount == 0)
    {
        ThreadHandler::unblockInterrupts();
    }
}

Thread::Thread(int8_t priority, int32_t period, uint32_t startOffset) :
    priority(priority),
    period(period),
    startOffset(startOffset)
{
    ThreadHandler::getInstance()->add(this);
}

Thread::~Thread()
{
    ThreadHandler::getInstance()->remove(this);
}

void Thread::initiate(uint32_t currentTime)
{
    if (!initiated)
    {
        runAtTimestamp = currentTime + startOffset;
        startOffset = runAtTimestamp;
        initiated = true;
    }
}

void Thread::updateCurrentTime(uint32_t currentTime)
{
    initiate(currentTime);

    timeUntillRun = static_cast<int32_t>(runAtTimestamp - currentTime);

    if (timeUntillRun < -(static_cast<int32_t>(1) << 30))
    {
        runAtTimestamp += static_cast<uint32_t>(1) << 29;
    }
}

bool Thread::pendingRun()
{
    return timeUntillRun <= 0;
}

bool Thread::higherPriorityThan(const Thread* other)
{
    if (other == nullptr)
    {
        return true;
    }

    return higherPriorityThan(*other);
}

bool Thread::higherPriorityThan(const Thread& other)
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

void Thread::runThread()
{
    run();
    if (firstCodeBlock())
    {
        runAtTimestamp = startOffset + period;
        startOffset = runAtTimestamp;
    }

}

int8_t Thread::getPriority() const
{
    return priority;
}

bool Thread::inRunQueue() const
{
    ThreadInterruptBlocker blocker;

    return nextPendingRun != this ||
            ThreadHandler::getInstance()->currentThread == this;
}

void Thread::disableExecution()
{
    ThreadInterruptBlocker blocker;

    startOffset = -1;
}

void Thread::enableExecution(int32_t period, uint32_t startOffset)
{
    ThreadInterruptBlocker blocker;

    initiated = false;
    if (period != 0)
    {
        this->period = period;
    }
    this->startOffset = startOffset;
}

void Thread::delayNextCodeBlock(int32_t delay)
{
    ThreadHandler::getInstance()->delayNextCodeBlock(delay);
}

void Thread::delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
    ThreadHandler::getInstance()->delayNextCodeBlockUntil(fun);
}

uint32_t Thread::getTimingError()
{
    return ThreadHandler::getInstance()->getTimingError();
}

void Thread::internalDelayNextCodeBlock(int32_t delay)
{
}

void Thread::internalDelayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
}

uint32_t Thread::internalGetTimingError()
{
    return ThreadHandler::syncedMicros() - runAtTimestamp;
}

bool Thread::firstCodeBlock()
{
    return true;
}

bool Thread::splitIntoCodeBlocks()
{
    return false;
}

ThreadHandler* ThreadHandler::getInstance()
{
    return createAndConfigureThreadHandler();
}

ThreadHandler::ThreadHandler()
{
}

ThreadHandler::~ThreadHandler()
{
}

int8_t ThreadHandler::getExecutionHaltedOnPriorityAfterDelete()
{
    ThreadInterruptBlocker blocker;

    return executionHaltedOnPrio;
}

void ThreadHandler::add(Thread* t)
{
    ThreadInterruptBlocker blocker;

    if (firstThread == nullptr)
    {
        firstThread = t;
    }
    else
    {
        Thread* insertPoint = nullptr;
        for (Thread* it = firstThread; it != nullptr; it = it->next)
        {
            if (t->getPriority() > it->getPriority())
            {
                break;
            }
            insertPoint = it;
        }

        if (insertPoint != nullptr)
        {
            t->previous = insertPoint;
            t->next = insertPoint->next;

            insertPoint->next = t;
            if (t->next != nullptr)
            {
                t->next->previous = t;
            }
        }
        else
        {
            t->previous = nullptr;
            t->next = firstThread;

            t->next->previous = t;

            firstThread = t;
        }
    }
}

void ThreadHandler::remove(const Thread* t)
{
    ThreadInterruptBlocker blocker;

    bool halt = t->inRunQueue();
    if (halt)
    {
        executionHaltedOnPrio = t->getPriority();
        blocker.unlock();

        while (true)
        {
        }
    }

    if (firstThread == t)
    {
        firstThread = t->next;

        t->next->previous = t->previous;
    }
    else
    {
        t->previous->next = t->next;
        if (t->next != nullptr)
        {
            t->next->previous = t->previous;
        }
    }
}

Thread* ThreadHandler::getHeadOfThreadsToRun(uint32_t currentTimestamp)
{
    Thread* headThreadToRun = nullptr;
    Thread* lastThreadToRun = nullptr;

    for (Thread* it = firstThread; it != nullptr; it = it->next)
    {
        if (it->getPriority() <= priorityOfRunningThread ||
            it->startOffset == -1)
        {
            continue;
        }

        it->updateCurrentTime(currentTimestamp);

        if (it->pendingRun())
        {
            it->nextPendingRun = nullptr;

            if (headThreadToRun == nullptr)
            {
                headThreadToRun = it;
                lastThreadToRun = it;
            }
            else
            {
                lastThreadToRun->nextPendingRun = it;
                lastThreadToRun = it;
            }
        }
    }

    return headThreadToRun;
}

void ThreadHandler::enableThreadExecution(bool enable)
{
    interruptTimer = getInterruptTimerInstance();

    ThreadInterruptBlocker blocker;

    threadExecutionEnabled = enable;
}

uint8_t ThreadHandler::getCpuLoad()
{
    ThreadInterruptBlocker blocker;

    uint8_t out = static_cast<uint8_t>((cpuLoadTime * 10ul) / (totalTime / 10));
    return out;
}

void ThreadHandler::delayNextCodeBlock(int32_t delay)
{
    currentThread->internalDelayNextCodeBlock(delay);
}

void ThreadHandler::delayNextCodeBlockUntil(FunctionalWrapper<bool>* fun)
{
    currentThread->internalDelayNextCodeBlockUntil(fun);
}

uint32_t ThreadHandler::getTimingError()
{
    return currentThread->internalGetTimingError();
}

Thread* ThreadHandler::getNextThreadToRunAndRemoveFrom(Thread*& head)
{
    Thread* highestPriorityParrent = nullptr;
    Thread* highestPriority = nullptr;

    Thread* prevIt = nullptr;
    uint8_t maxPriority = head->getPriority();

    for (Thread* it = head; it != nullptr; it = it->nextPendingRun)
    {
        if (it->getPriority() < maxPriority)
        {
            break;
        }

        if (it->higherPriorityThan(highestPriority))
        {
            highestPriorityParrent = prevIt;
            highestPriority = it;
        }

        prevIt = it;
    }

    if (highestPriorityParrent == nullptr)
    {
        head = highestPriority->nextPendingRun;
        highestPriority->nextPendingRun = highestPriority;
    }
    else
    {
        highestPriorityParrent->nextPendingRun = highestPriority->nextPendingRun;
        highestPriority->nextPendingRun = highestPriority;
    }

    return highestPriority;
}


void ThreadHandler::InterruptTimerInterface::interruptRun()
{
    ThreadHandler::getInstance()->interruptRun();
}

void ThreadHandler::interruptRun()
{
    if (!threadExecutionEnabled)
    {
        return;
    }

    ThreadInterruptBlocker blocker;
    ThreadHandler::enableNewInterrupt();

    uint32_t currentTimestamp = ThreadHandler::getInterruptTimestamp();

    static uint32_t startTime = currentTimestamp;
    static uint32_t loadStartTime = startTime;

    if (priorityOfRunningThread == -128)
    {
        loadStartTime = ThreadHandler::syncedMicros();
    }

    Thread* headThreadToRun = getHeadOfThreadsToRun(currentTimestamp);

    while (true)
    {
        Thread* threadToRun = getNextThreadToRunAndRemoveFrom(headThreadToRun);

        if (threadToRun != nullptr)
        {
            int8_t temp = priorityOfRunningThread;
            auto temp2 = currentThread;
            priorityOfRunningThread = threadToRun->getPriority();
            currentThread = threadToRun;
            blocker.unlock();

            threadToRun->runThread();

            blocker.lock();
            currentThread = temp2;
            priorityOfRunningThread = temp;
        }
        if (threadToRun == nullptr)
        {
            if (priorityOfRunningThread == -128)
            {
                uint32_t endTime = ThreadHandler::syncedMicros();
                int32_t timeDiff = static_cast<int32_t>(endTime - startTime);
                int32_t loadTimeDiff = static_cast<int32_t>(endTime - loadStartTime);
                startTime = endTime;

                totalTime = (totalTime * 15 + timeDiff) / 16;
                cpuLoadTime = (cpuLoadTime * 15 + loadTimeDiff) / 16;
            }
            break;
        }
    }
}

template<>
void ThreadHandler::callBlock<ThreadHandler::InterruptTimerInterface>()
{
    interruptTimer->blockInterrupts();
}
template<>
void ThreadHandler::callUnblock<ThreadHandler::InterruptTimerInterface>()
{
    interruptTimer->unblockInterrupts();
}

template<>
void ThreadHandler::callEnableNewInterrupt<ThreadHandler::InterruptTimerInterface>()
{
    interruptTimer->enableNewInterrupt();
}
template<>
uint32_t ThreadHandler::callSyncedMicros<ThreadHandler::InterruptTimerInterface>()
{
    return interruptTimer->syncedMicros();
}

template<>
uint32_t ThreadHandler::callGetInterruptTimestamp<ThreadHandler::InterruptTimerInterface>()
{
    return interruptTimer->getInterruptTimestamp();
}

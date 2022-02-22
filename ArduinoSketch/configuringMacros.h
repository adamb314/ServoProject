#define THREAD_HANDLER(InterruptTimer) \
ThreadHandler::InterruptTimerInterface* getInterruptTimerInstance() \
{ \
    return InterruptTimer; \
} \
 \
void ThreadHandler::blockInterrupts() \
{ \
    callBlock<decltype(getClassType(InterruptTimer))>(); \
} \
 \
void ThreadHandler::unblockInterrupts() \
{ \
    callUnblock<decltype(getClassType(InterruptTimer))>(); \
} \
 \
void ThreadHandler::enableNewInterrupt() \
{ \
    callEnableNewInterrupt<decltype(getClassType(InterruptTimer))>(); \
} \
 \
uint32_t ThreadHandler::syncedMicros() \
{ \
    return callSyncedMicros<decltype(getClassType(InterruptTimer))>(); \
} \
 \
uint32_t ThreadHandler::getInterruptTimestamp() \
{ \
    return callGetInterruptTimestamp<decltype(getClassType(InterruptTimer))>(); \
}

#define THREAD_HANDLER_WITH_EXECUTION_ORDER_OPTIMIZED(InterruptTimer) THREAD_HANDLER(InterruptTimer)

#define SET_THREAD_HANDLER_TICK(InterruptTimerTick) \
uint32_t getInterruptTimerTick() \
{ \
    return InterruptTimerTick; \
}

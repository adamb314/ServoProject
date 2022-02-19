#include "ThreadHandler.h"

extern uint32_t getInterruptTimerTick();

#if defined(_SAMD21_)

uint16_t InterruptTimer::interruptTick;

InterruptTimer* InterruptTimer::getInstance()
{
    static InterruptTimer inst(getInterruptTimerTick());
    return &inst;
}

InterruptTimer::InterruptTimer(uint16_t period)
{
    if (period == 0)
    {
        period = 1000;
    }
    if (period < 10)
    {
        period = 10;
    }
    interruptTick = period;
    configure(period);
    startCounter();
}

void InterruptTimer::enableNewInterrupt()
{
    enableNewInterruptImp();
}

void InterruptTimer::blockInterrupts()
{
    blockInterruptsImp();
}

void InterruptTimer::unblockInterrupts()
{
    unblockInterruptsImp();
}

uint32_t InterruptTimer::getInterruptTimestamp()
{
    return getInterruptTimestampImp();
}

uint32_t InterruptTimer::syncedMicros()
{
    return syncedMicrosImp();
}

void InterruptTimer::enableNewInterruptImp()
{
    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
}

void InterruptTimer::blockInterruptsImp()
{
    NVIC_DisableIRQ(TC5_IRQn);
}

void InterruptTimer::unblockInterruptsImp()
{
    NVIC_EnableIRQ(TC5_IRQn);
}

uint32_t InterruptTimer::getInterruptTimestampImp()
{
    return interruptTimerTime;
}

uint32_t InterruptTimer::syncedMicrosImp()
{
    return micros() - microsTimerOffset;
}

uint32_t InterruptTimer::interruptTimerTime = 0;
uint32_t InterruptTimer::microsTimerOffset = 0;

void InterruptTimer::interruptRun()
{
    interruptTimerTime += interruptTick;

    static uint8_t interruptLevel = 0;

    blockInterruptsImp();
    interruptLevel++;

    if (interruptLevel == 1)
    {
        uint32_t newMicrosTimerOffset = micros() - interruptTimerTime;

        while (static_cast<int16_t>(newMicrosTimerOffset - microsTimerOffset) >
                static_cast<int16_t>(interruptTick / 2))
        {
            interruptTimerTime += interruptTick;
            newMicrosTimerOffset -= interruptTick;
        }

        microsTimerOffset = newMicrosTimerOffset;
    }
    unblockInterruptsImp();

    ThreadHandler::InterruptTimerInterface::interruptRun();

    blockInterruptsImp();
    interruptLevel--;
    unblockInterruptsImp();
}

void InterruptTimer::configure(uint16_t period)
{
    if (period != 0)
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
    }

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
bool InterruptTimer::isSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void InterruptTimer::startCounter()
{
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
    while (isSyncing()); //wait until snyc'd
}

//Reset TC5 
void InterruptTimer::reset()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (isSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void InterruptTimer::disable()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (isSyncing());
}

extern "C"
{
void tc5InterruptRunCaller()
{
    InterruptTimer::interruptRun();
}

__attribute__((naked))
void TC5_Handler()
{
    asm volatile(
    // Now we are in Handler mode, using Main Stack, and
    // SP should be Double word aligned
    "  PUSH {R4, LR}       \n"// Need to save LR in stack, keep double word alignment
    );

    TC5->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;

    asm volatile(
    "  SUB  SP, SP , #0x20 \n"// Reserve 8 words for dummy stack frame for return
    "  MOV  R2, SP\n"
    "  LDR  R3,=SysTick_Handler_thread_pt\n"
    "  STR  R3,[R2, #24]   \n"// Set return address as SysTick_Handler_thread_pt
    "  LDR  R3,=0x01000000 \n"// Initial xPSR when running Reentrant_SysTick_Handler
    "  STR  R3,[R2, #28]   \n"// Put in new created stack frame
    "  LDR  R3,=0xFFFFFFF9 \n"// Return to Thread with Main Stack
    "  BX   R3             \n"// Exception return with new created stack frame
    "SysTick_Handler_thread_pt:\n"
    );

    asm volatile(
    "  BL   tc5InterruptRunCaller \n"// Call real ISR in thread mode
    "  SVC  0              \n"// Use SVC to return to original Thread
    "  B    .              \n"// Should not return here
    "  .align 4"
    );
}

// SVC handler - restore stack
//__attribute__((naked))
void SVC_Handler(void)
{
    asm volatile(
    "  MOVS   r0, #4\n"
    "  MOV    r1, LR\n"
    "  TST    r0, r1\n"
    "  BEQ    stacking_used_MSP\n"
    "  MRS    R0, PSP \n"// first parameter - stacking was using PSP
    "  B      get_SVC_num\n"
    "stacking_used_MSP:\n"
    "  MRS    R0, MSP \n"// first parameter - stacking was using MSP
    "get_SVC_num:\n"
    "  LDR     R1, [R0, #24]  \n"// Get stacked PC
    "  SUB    R1, R1, #2\n"
    "  LDRB    R0, [R1, #0]   \n"// Get SVC parameter at stacked PC minus 2
    "  CMP     R0, #0\n"
    "  BEQ     svc_service_0\n"
    "  BL      Unknown_SVC_Request\n"
    "  BX      LR \n"// return
    "svc_service_0:\n"
    // SVC service 0
    // Reentrant code finished, we can discard the current stack frame
    // and restore the original stack frame.
    "  ADD     SP, SP, #0x20\n"
    "  POP     {R4, PC} \n"// Return
    "  .align 4\n"
    );
}
//--------------------------------------
void Unknown_SVC_Request(unsigned int svc_num)
{  //Display Error Message when SVC service is not known
    printf("Error: Unknown SVC service request %d\n", svc_num);
    while(1);
}

}

#elif defined(__AVR__)

InterruptTimer* InterruptTimer::getInstance()
{
    static InterruptTimer inst(getInterruptTimerTick());
    return &inst;
}

InterruptTimer::InterruptTimer(uint16_t interruptTick)
{
    if (interruptTick != 0)
    {
        Timer1.initialize(interruptTick);
    }
    Timer1.attachInterrupt(interruptHandler);
}

void InterruptTimer::enableNewInterrupt()
{
    enableNewInterruptImp();
}

void InterruptTimer::blockInterrupts()
{
    blockInterruptsImp();
}

void InterruptTimer::unblockInterrupts()
{
    unblockInterruptsImp();
}

uint32_t InterruptTimer::microsAtInterrupt = 0;

void InterruptTimer::interruptRun()
{
    microsAtInterrupt = micros();
    ThreadHandler::InterruptTimerInterface::interruptRun();
}

uint32_t InterruptTimer::getInterruptTimestamp()
{
    return getInterruptTimestampImp();
}

uint32_t InterruptTimer::syncedMicros()
{
    return syncedMicrosImp();
}

void interruptHandler() {
    interrupts();
    InterruptTimer::interruptRun();
}

void InterruptTimer::enableNewInterruptImp()
{
    TIFR1 = _BV(TOV1);
}

void InterruptTimer::blockInterruptsImp()
{
    TIMSK1 = 0;
}

void InterruptTimer::unblockInterruptsImp()
{
    TIMSK1 = _BV(TOIE1);
}

uint32_t InterruptTimer::getInterruptTimestampImp()
{
    return microsAtInterrupt;
}

uint32_t InterruptTimer::syncedMicrosImp()
{
    return micros();
}

#endif

template<>
void ThreadHandler::callBlock<InterruptTimer>()
{
    InterruptTimer::blockInterruptsImp();
}
template<>
void ThreadHandler::callUnblock<InterruptTimer>()
{
    InterruptTimer::unblockInterruptsImp();
}

template<>
void ThreadHandler::callEnableNewInterrupt<InterruptTimer>()
{
    InterruptTimer::enableNewInterruptImp();
}
template<>
uint32_t ThreadHandler::callSyncedMicros<InterruptTimer>()
{
    return InterruptTimer::syncedMicrosImp();
}

template<>
uint32_t ThreadHandler::callGetInterruptTimestamp<InterruptTimer>()
{
    return InterruptTimer::getInterruptTimestampImp();
}

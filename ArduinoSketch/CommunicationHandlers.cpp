#include "CommunicationHandlers.h"

static ThreadHandler* threadHandler = nullptr;

DCServoCommunicationHandler::DCServoCommunicationHandler(unsigned char nodeNr, std::unique_ptr<DCServo> dcServo) :
    CommunicationNode(nodeNr), dcServo(std::move(dcServo))
{
    threadHandler = ThreadHandler::getInstance();
    CommunicationNode::intArray[0] = this->dcServo->getPosition() * positionUpscaling;
    CommunicationNode::intArray[1] = 0;
    CommunicationNode::intArray[2] = 0;

    intArrayIndex0Upscaler.set(CommunicationNode::intArray[0]);
}

DCServoCommunicationHandler::~DCServoCommunicationHandler()
{
}

void DCServoCommunicationHandler::onReadyToSendEvent()
{
}

void DCServoCommunicationHandler::onReceiveCompleteEvent()
{
    dcServo->onlyUseMainEncoder(CommunicationNode::charArray[2] == 1);

    if (CommunicationNode::charArrayChanged[3] ||
        CommunicationNode::charArrayChanged[4] ||
        CommunicationNode::charArrayChanged[5])
    {
        if (!CommunicationNode::charArrayChanged[4] &&
                !CommunicationNode::charArrayChanged[5])
        {
            CommunicationNode::charArray[4] = CommunicationNode::charArray[3];
            CommunicationNode::charArray[5] = CommunicationNode::charArray[3];
        }
        dcServo->setControlSpeed(CommunicationNode::charArray[3],
                CommunicationNode::charArray[4] * 4,
                CommunicationNode::charArray[5] * 32);
    }

    if (CommunicationNode::charArrayChanged[6] ||
        CommunicationNode::charArrayChanged[7] ||
        CommunicationNode::charArrayChanged[8])
    {
        dcServo->setBacklashControlSpeed(CommunicationNode::charArray[6],
                CommunicationNode::charArray[7],
                CommunicationNode::charArray[8]);
    }

    if (CommunicationNode::intArrayChanged[0])
    {
        intArrayIndex0Upscaler.update(CommunicationNode::intArray[0]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0 / positionUpscaling), CommunicationNode::intArray[1], CommunicationNode::intArray[2]);

        CommunicationNode::intArrayChanged[0] = false;
        dcServo->openLoopMode(false);
        dcServo->enable(true);

        statusLight.showEnabled();
    }
    else if (CommunicationNode::intArrayChanged[2])
    {
        intArrayIndex0Upscaler.set(CommunicationNode::intArray[3]);
        dcServo->loadNewReference(intArrayIndex0Upscaler.get() * (1.0 / positionUpscaling), 0.0, CommunicationNode::intArray[2]);

        CommunicationNode::intArrayChanged[2] = false;
        dcServo->openLoopMode(true, CommunicationNode::charArray[1] == 1);
        dcServo->enable(true);

        statusLight.showOpenLoop();
    }
    else
    {
        dcServo->enable(false);

        intArrayIndex0Upscaler.set(CommunicationNode::intArray[3]);

        statusLight.showDisabled();
    }
}

void DCServoCommunicationHandler::onErrorEvent()
{
}

void DCServoCommunicationHandler::onComCycleEvent()
{
    {
        ThreadInterruptBlocker blocker;

        long int pos = dcServo->getPosition() * positionUpscaling;
        CommunicationNode::intArray[3] = pos;
        CommunicationNode::charArray[9] = static_cast<char>(pos >> 16);
        CommunicationNode::intArray[4] = dcServo->getVelocity();
        CommunicationNode::intArray[5] = dcServo->getControlSignal();
        CommunicationNode::intArray[6] = dcServo->getCurrent();
        CommunicationNode::intArray[7] = dcServo->getPwmControlSignal();
        CommunicationNode::intArray[8] = threadHandler->getCpuLoad();
        CommunicationNode::intArray[9] = dcServo->getLoopNumber();
        CommunicationNode::intArray[10] = dcServo->getMainEncoderPosition() * positionUpscaling;
        CommunicationNode::intArray[11] = dcServo->getBacklashCompensation() * positionUpscaling;

        auto opticalEncoderChannelData = dcServo->getMainEncoderDiagnosticData();
        CommunicationNode::intArray[12] = opticalEncoderChannelData.a;
        CommunicationNode::intArray[13] = opticalEncoderChannelData.b;
        CommunicationNode::intArray[14] = opticalEncoderChannelData.c;
        CommunicationNode::intArray[15] = opticalEncoderChannelData.d;

        dcServo->triggerReferenceTiming();
    }

    statusLight.showCommunicationActive();
}

void DCServoCommunicationHandler::onComIdleEvent()
{
    CommunicationNode::intArrayChanged[0] = false;
    CommunicationNode::intArrayChanged[2] = false;
    dcServo->enable(false);

    statusLight.showDisabled();
    statusLight.showCommunicationInactive();
}

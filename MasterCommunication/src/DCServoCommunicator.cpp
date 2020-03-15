#include "DCServoCommunicator.h"

DCServoCommunicator::DCServoCommunicator(unsigned char nodeNr, Communication* bus) :
        activeIntReads{false}
{
    this->nodeNr = nodeNr;
    this->bus = bus;

    communicationIsOk = false;
    initState = 0;
    backlashControlDisabled = false;
    newPositionReference = false;
    newOpenLoopControlSignal = false;

    setOffsetAndScaling(1.0, 0);
}

void DCServoCommunicator::setOffsetAndScaling(double scale, double offset)
{
	this->scale = scale;
	this->offset = offset;

    if (isInitComplete())
    {
        float pos = getPosition() / scale;

        if (pos > 2048)
        {
            this->offset -= 4096 * scale;
        }
        else if (pos < -2048)
        {
            this->offset += 4096 * scale;
        }
    }
}

void DCServoCommunicator::disableBacklashControl(bool b)
{
    backlashControlDisabled = b;
}

bool DCServoCommunicator::isInitComplete()
{
    return initState == 10;
}

bool DCServoCommunicator::isCommunicationOk()
{
    return communicationIsOk;
}

void DCServoCommunicator::setReference(const float& pos, const float& vel, const float& feedforwardU)
{
    newPositionReference = true;
    newOpenLoopControlSignal = false;
    refPos = (pos - offset) / scale * 4;
    refVel = vel / scale;
    this->feedforwardU = feedforwardU;
}

void DCServoCommunicator::setOpenLoopControlSignal(const float& feedforwardU, bool pwmMode)
{
    newOpenLoopControlSignal = true;
    newPositionReference = false;
    pwmOpenLoopMode = pwmMode;
    this->feedforwardU = feedforwardU;
}

float DCServoCommunicator::getPosition(bool withBacklash)
{
    float pos;
    if (withBacklash && !backlashControlDisabled)
    {
        activeIntReads[3] = true;
        pos = backlashEncoderPos;
    }
    else
    {
        activeIntReads[9] = true;
        pos = encoderPos;
    }

    return scale * pos + offset;
}

float DCServoCommunicator::getVelocity()
{
    activeIntReads[4] = true;
    return scale * encoderVel;
}

float DCServoCommunicator::getControlSignal()
{
    activeIntReads[5] = true;
    return controlSignal;
}

float DCServoCommunicator::getFeedforwardU()
{
    return activeFeedforwardU[2];
}

float DCServoCommunicator::getControlError()
{
    float pos;
    if (!backlashControlDisabled)
    {
        activeIntReads[3] = true;
        pos = backlashEncoderPos;
    }
    else
    {
        activeIntReads[9] = true;
        pos = encoderPos;
    }

    return scale * (activeRefPos[2] * 0.25 - pos);
}

float DCServoCommunicator::getCurrent()
{
    activeIntReads[6] = true;
    return current;
}

int DCServoCommunicator::getCpuLoad()
{
    activeIntReads[7] = true;
    return cpuLoad;
}

int DCServoCommunicator::getLoopTime()
{
    activeIntReads[8] = true;
    return loopTime;
}

DCServoCommunicator::OpticalEncoderChannelData DCServoCommunicator::getOpticalEncoderChannelData()
{
    activeIntReads[12] = true;
    activeIntReads[13] = true;
    activeIntReads[14] = true;
    activeIntReads[15] = true;
    return opticalEncoderChannelData;
}

void DCServoCommunicator::run()
{
    bus->setNodeNr(nodeNr);

    for (size_t i = 0; i < activeIntReads.size(); i++)
    {
        if (activeIntReads[i] || !isInitComplete())
        {
            activeIntReads[i] = false;
            bus->requestReadInt(i);
        }
    }

    if (isInitComplete())
    {
        if (newPositionReference)
        {
            bus->write(0, refPos);
            bus->write(1, refVel);
            bus->write(2, feedforwardU);

            activeRefPos[4] = activeRefPos[3];
            activeRefPos[3] = activeRefPos[2];
            activeRefPos[2] = activeRefPos[1];
            activeRefPos[1] = activeRefPos[0];
            activeRefPos[0] = refPos;

            newPositionReference = false;
        }
        else if (newOpenLoopControlSignal)
        {
            bus->write(2, feedforwardU);
            bus->write(1, static_cast<char>(pwmOpenLoopMode));

            newOpenLoopControlSignal = false;
        }

        activeFeedforwardU[4] = activeFeedforwardU[3];
        activeFeedforwardU[3] = activeFeedforwardU[2];
        activeFeedforwardU[2] = activeFeedforwardU[1];
        activeFeedforwardU[1] = activeFeedforwardU[0];
        activeFeedforwardU[0] = feedforwardU;
    }
    else
    {
        bus->write(2, static_cast<char>(backlashControlDisabled));
    }

    communicationIsOk = bus->execute();

    if (communicationIsOk)
    {
        if (initState < 10)
        {
            initState++;

            float pos;
            if (!backlashControlDisabled)
            {
                pos = backlashEncoderPos;
            }
            else
            {
                pos = encoderPos;
            }

            activeRefPos[0] = pos * 4;
            activeRefPos[1] = activeRefPos[0];
            activeRefPos[2] = activeRefPos[1];
            activeRefPos[3] = activeRefPos[2];
            activeRefPos[4] = activeRefPos[3];
        }
        
        backlashEncoderPos = bus->getLastReadInt(3) * 0.25;
        encoderPos = bus->getLastReadInt(9) * 0.25;
        encoderVel = bus->getLastReadInt(4);
        controlSignal = bus->getLastReadInt(5);
        current = bus->getLastReadInt(6);
        cpuLoad = bus->getLastReadInt(7);
        loopTime = bus->getLastReadInt(8);
        opticalEncoderChannelData.a = bus->getLastReadInt(12);
        opticalEncoderChannelData.b = bus->getLastReadInt(13);
        opticalEncoderChannelData.minCostIndex = bus->getLastReadInt(14);
        opticalEncoderChannelData.minCost = bus->getLastReadInt(15);
    }
}

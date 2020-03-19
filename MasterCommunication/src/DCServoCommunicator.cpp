#include "DCServoCommunicator.h"

DCServoCommunicator::DCServoCommunicator(unsigned char nodeNr, Communication* bus)
{
    activeIntReads.fill(true);
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
        activeIntReads[10] = true;
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

float DCServoCommunicator::getControlError(bool withBacklash)
{
    float pos;
    if (!backlashControlDisabled)
    {
        if (withBacklash)
        {
            activeIntReads[3] = true;
            pos = backlashEncoderPos;
        }
        else
        {
            activeIntReads[10] = true;
            activeIntReads[11] = true;
            pos = encoderPos + backlashCompensation;
        }
    }
    else
    {
        activeIntReads[10] = true;
        pos = encoderPos;
    }

    return scale * (activeRefPos[2] * 0.25 - pos);
}

float DCServoCommunicator::getCurrent()
{
    activeIntReads[6] = true;
    return current;
}

int DCServoCommunicator::getPwmControlSignal()
{
    activeIntReads[7] = true;
    return pwmControlSignal;
}

int DCServoCommunicator::getCpuLoad()
{
    activeIntReads[8] = true;
    return cpuLoad;
}

int DCServoCommunicator::getLoopTime()
{
    activeIntReads[9] = true;
    return loopTime;
}

float DCServoCommunicator::getBacklashCompensation()
{
    activeIntReads[11] = true;
    return backlashCompensation;
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
        if (activeIntReads[i])
        {
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
        for (size_t i = 0; i < activeIntReads.size(); i++)
        {
            if (activeIntReads[i])
            {

                if (isInitComplete())
                {
                    activeIntReads[i] = false;
                }
                intReadBuffer[i] = bus->getLastReadInt(i);
            }
        }
        
        backlashEncoderPos = intReadBuffer[3] * 0.25;
        encoderPos = intReadBuffer[10] * 0.25;
        backlashCompensation = intReadBuffer[11] * 0.25;
        encoderVel = intReadBuffer[4];
        controlSignal = intReadBuffer[5];
        current = intReadBuffer[6];
        pwmControlSignal = intReadBuffer[7];
        cpuLoad = intReadBuffer[8];
        loopTime = intReadBuffer[9];
        opticalEncoderChannelData.a = intReadBuffer[12];
        opticalEncoderChannelData.b = intReadBuffer[13];
        opticalEncoderChannelData.minCostIndex = intReadBuffer[14];
        opticalEncoderChannelData.minCost = intReadBuffer[15];

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
    }
}

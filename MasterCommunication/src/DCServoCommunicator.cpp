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

void DCServoCommunicator::setOffsetAndScaling(double scale, double offset, double startPosition)
{
	this->scale = scale;
	this->offset = offset;
    this->startPosition = startPosition;

    if (isInitComplete())
    {
        updateOffset();
    }
}

void DCServoCommunicator::updateOffset()
{
    float pos = getPosition() / scale;
    startPosition /= scale;

    if (pos - startPosition > (2048 / 2))
    {
        offset -= (4096 / 2) * scale;
    }
    else if (pos - startPosition < -(2048 / 2))
    {
        offset += (4096 / 2) * scale;
    }
}

void DCServoCommunicator::setControlSpeed(unsigned char controlSpeed)
{
    this->controlSpeed = controlSpeed;
}

void DCServoCommunicator::setBacklashControlSpeed(unsigned char backlashCompensationSpeed,
            double backlashCompensationCutOffSpeed,
            double backlashSize)
{
    this->backlashCompensationSpeed = backlashCompensationSpeed;
    this->backlashCompensationSpeedVelDecrease = static_cast<unsigned char>(std::min(255.0,
            255 * 10 / (backlashCompensationCutOffSpeed / std::abs(scale))));
    this->backlashSize = static_cast<unsigned char>(backlashSize / std::abs(scale));
}

void DCServoCommunicator::setFrictionCompensation(double fricComp)
{
    this->frictionCompensation = fricComp;
}


void DCServoCommunicator::disableBacklashControl(bool b)
{
    backlashControlDisabled = b;
}

bool DCServoCommunicator::isInitComplete() const
{
    return initState == 10;
}

bool DCServoCommunicator::isCommunicationOk() const
{
    return communicationIsOk;
}

void DCServoCommunicator::setReference(const float& pos, const float& vel, const float& feedforwardU)
{
    newPositionReference = true;
    newOpenLoopControlSignal = false;
    refPos = (pos - offset) / scale * positionUpscaling;
    refVel = vel / scale;

    if (refVel > 4)
    {
        frictionCompensation = std::abs(frictionCompensation);
    }
    else if (refVel < -4)
    {
        frictionCompensation = -std::abs(frictionCompensation);
    }
    this->feedforwardU = feedforwardU + frictionCompensation;
}

void DCServoCommunicator::setOpenLoopControlSignal(const float& feedforwardU, bool pwmMode)
{
    newOpenLoopControlSignal = true;
    newPositionReference = false;
    pwmOpenLoopMode = pwmMode;
    this->feedforwardU = feedforwardU;
}

float DCServoCommunicator::getPosition(bool withBacklash) const
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

float DCServoCommunicator::getVelocity() const
{
    activeIntReads[4] = true;
    return scale * encoderVel;
}

float DCServoCommunicator::getControlSignal() const
{
    activeIntReads[5] = true;
    return controlSignal;
}

float DCServoCommunicator::getFeedforwardU() const
{
    return activeFeedforwardU[2];
}

float DCServoCommunicator::getControlError(bool withBacklash) const
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

    return scale * (activeRefPos[2] * (1.0 / positionUpscaling) - pos);
}

float DCServoCommunicator::getCurrent() const
{
    activeIntReads[6] = true;
    return current;
}

short int DCServoCommunicator::getPwmControlSignal() const
{
    activeIntReads[7] = true;
    return pwmControlSignal;
}

short int DCServoCommunicator::getCpuLoad() const
{
    activeIntReads[8] = true;
    return cpuLoad;
}

short int DCServoCommunicator::getLoopTime() const
{
    activeIntReads[9] = true;
    return loopTime;
}

float DCServoCommunicator::getBacklashCompensation() const
{
    activeIntReads[11] = true;
    return scale * backlashCompensation;
}

DCServoCommunicator::OpticalEncoderChannelData DCServoCommunicator::getOpticalEncoderChannelData() const
{
    activeIntReads[12] = true;
    activeIntReads[13] = true;
    activeIntReads[14] = true;
    activeIntReads[15] = true;
    return opticalEncoderChannelData;
}

double DCServoCommunicator::getScaling()
{
    return scale;
}

double DCServoCommunicator::getOffset()
{
    return offset;
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
            bus->write(0, static_cast<short int>(refPos));
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

        bus->write(3, static_cast<char>(controlSpeed));
        bus->write(4, static_cast<char>(backlashCompensationSpeed));
        bus->write(5, static_cast<char>(backlashCompensationSpeedVelDecrease));
        bus->write(6, static_cast<char>(backlashSize));
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

        if (isInitComplete())
        {
            intReadBufferIndex3Upscaling.update(intReadBuffer[3]);
            intReadBufferIndex10Upscaling.update(intReadBuffer[10]);
            intReadBufferIndex11Upscaling.update(intReadBuffer[11]);
        }
        else
        {
            intReadBufferIndex3Upscaling.set(intReadBuffer[3]);
            intReadBufferIndex10Upscaling.set(intReadBuffer[10]);
            intReadBufferIndex11Upscaling.set(intReadBuffer[11]);
        }
        
        backlashEncoderPos = intReadBufferIndex3Upscaling.get() * (1.0 / positionUpscaling);
        encoderPos = intReadBufferIndex10Upscaling.get() * (1.0 / positionUpscaling);
        backlashCompensation = intReadBufferIndex11Upscaling.get() * (1.0 / positionUpscaling);

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

        if (!isInitComplete())
        {
            ++initState;

            float pos;
            if (!backlashControlDisabled)
            {
                pos = backlashEncoderPos;
            }
            else
            {
                pos = encoderPos;
            }

            activeRefPos[0] = pos * positionUpscaling;
            activeRefPos[1] = activeRefPos[0];
            activeRefPos[2] = activeRefPos[1];
            activeRefPos[3] = activeRefPos[2];
            activeRefPos[4] = activeRefPos[3];

            if (isInitComplete())
            {
                updateOffset();
            }
        }
    }
}

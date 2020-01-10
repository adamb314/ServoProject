#include "DCServoCommunicator.h"

DCServoCommunicator::DCServoCommunicator(unsigned char nodeNr, Communication* bus) :
        activeIntReads{false}
{
    this->nodeNr = nodeNr;
    this->bus = bus;

    communicationIsOk = false;
    initState = 0;
    newReference = false;

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
    newReference = true;
    refPos = (pos - offset) / scale * 4;
    refVel = vel / scale;
    this->feedforwardU = feedforwardU;
}

float DCServoCommunicator::getPosition()
{
    activeIntReads[3] = true;
    return scale * encoderPos + offset;
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

float DCServoCommunicator::getControlError()
{
    activeIntReads[3] = true;
    return scale * (activeRefPos[2] * 0.25 - encoderPos);
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

    if (isInitComplete() && newReference)
    {
        newReference = false;
        bus->write(0, refPos);
        bus->write(1, refVel);
        bus->write(2, feedforwardU);

        activeRefPos[4] = activeRefPos[3];
        activeRefPos[3] = activeRefPos[2];
        activeRefPos[2] = activeRefPos[1];
        activeRefPos[1] = activeRefPos[0];
        activeRefPos[0] = refPos;
    }


    communicationIsOk = bus->execute();

    if (communicationIsOk)
    {
        if (initState < 10)
        {
            initState++;

            activeRefPos[0] = encoderPos * 4;
            activeRefPos[1] = activeRefPos[0];
            activeRefPos[2] = activeRefPos[1];
            activeRefPos[3] = activeRefPos[2];
            activeRefPos[4] = activeRefPos[3];

            bus->write(2, static_cast<char>(0));
        }

        encoderPos = bus->getLastReadInt(3) * 0.25;
        encoderVel = bus->getLastReadInt(4);
        controlSignal = bus->getLastReadInt(5);
        current = bus->getLastReadInt(6);
        cpuLoad = bus->getLastReadInt(7);
        loopTime = bus->getLastReadInt(8);
    }
}

#include "DCServoCommunicator.h"

DCServoCommunicator::DCServoCommunicator(unsigned char nodeNr, Communication* bus)
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
    return scale * encoderPos + offset;
}

float DCServoCommunicator::getVelocity()
{
    return scale * encoderVel;
}

float DCServoCommunicator::getControlSignal()
{
    return controlSignal;
}

float DCServoCommunicator::getControlError()
{
    return scale * (activeRefPos[2] * 0.25 - encoderPos);
}

float DCServoCommunicator::getCurrent()
{
    return current;
}

int DCServoCommunicator::getCpuLoad()
{
    return cpuLoad;
}

int DCServoCommunicator::getLoopTime()
{
    return loopTime;
}

bool DCServoCommunicator::runModelIdentTest(unsigned char testSequenceNumber, unsigned int amplitude)
{
    return modelIdentHandler.runModelIdentTest(testSequenceNumber, amplitude);
}

std::string DCServoCommunicator::getRecordedModelIdentData()
{
    return modelIdentHandler.getRecordedData();
}

void DCServoCommunicator::run()
{
    bus->setNodeNr(nodeNr);

    bus->requestReadInt(3);
    bus->requestReadInt(4);
    bus->requestReadInt(5);
    bus->requestReadInt(6);
    bus->requestReadInt(7);
    bus->requestReadInt(8);

    if (modelIdentHandler.activeRecording())
    {
        if (isInitComplete())
        {
            modelIdentHandler.handleWrite(bus);
        }
    }
    else
    {
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

        modelIdentHandler.handleRead(bus, round(encoderPos), controlSignal, current, loopTime);
    }
}

DCServoCommunicator::ModelIdentHandler::ModelIdentHandler() :
    runModelIdentState(0)
{
}

bool DCServoCommunicator::ModelIdentHandler::runModelIdentTest(unsigned char testSequenceNumber, unsigned int amplitude)
{
    this->amplitude = amplitude;
    this->testSequenceNumber = testSequenceNumber;

    if (runModelIdentState == 0)
    {
        runModelIdentState = 1;
    }
    else if (runModelIdentState == 100)
    {
        runModelIdentState = 0;
        return true;
    }

    return false;
}

std::string DCServoCommunicator::ModelIdentHandler::getRecordedData()
{
    return dataBuilder.str();
}

bool DCServoCommunicator::ModelIdentHandler::activeRecording()
{
    return runModelIdentState != 0;
}

void DCServoCommunicator::ModelIdentHandler::handleWrite(Communication* bus)
{
    if (runModelIdentState == 1)
    {
        dataBuilder.str("");
        bus->write(2, static_cast<int>(amplitude));
        bus->write(1, static_cast<char>(testSequenceNumber));
        bus->requestReadChar(1);
        runModelIdentState = 2;
    }
    else
    {
        bus->requestReadChar(1);
    }
}

void DCServoCommunicator::ModelIdentHandler::handleRead(Communication* bus, int encoderPos, int controlSignal, int current, int loopTime)
{
    if (runModelIdentState == 2)
    {
        if (bus->getLastReadChar(1) == testSequenceNumber)
        {
            runModelIdentState = 3;
            lastLoopTime = loopTime - 1;
        }
    }
    if (runModelIdentState == 3)
    {
        if (lastLoopTime != loopTime)
        {
            dataBuilder << static_cast<int16_t>(loopTime - lastLoopTime) <<
                    " " << controlSignal << " " << current << " " << encoderPos << "\n";
            lastLoopTime = loopTime;
        }
        if (bus->getLastReadChar(1) != testSequenceNumber)
        {
            runModelIdentState = 100;
        }
    }
}

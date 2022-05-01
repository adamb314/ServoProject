#include "ServoProjectController.h"

SerialCommunication::SerialCommunication(SerialComOptimizer serial, uint32_t timeout) :
        serial(serial), timeout(timeout)
{
}

SerialCommunication::SerialCommunication() :
        serial(nullptr), timeout(0)
{
}

void SerialCommunication::setNodeNr(unsigned char nr)
{
    nodeNr = nr;
}

void SerialCommunication::write(unsigned char nr, char value)
{
    commandArray.push_back(nr);
    commandArray.push_back(value);
}

void SerialCommunication::write(unsigned char nr, short int value)
{
    commandArray.push_back(nr + 64);
    commandArray.push_back(static_cast<unsigned char>(value));
    commandArray.push_back(static_cast<unsigned short>(value) / 256);
}

void SerialCommunication::requestReadChar(unsigned char nr)
{
    commandArray.push_back(nr + 128);
    receiveArray.push_back(nr);
}

void SerialCommunication::requestReadInt(unsigned char nr)
{
    commandArray.push_back(nr + 128 + 64);
    receiveArray.push_back(nr + 64);
}

char SerialCommunication::getLastReadChar(unsigned char nr)
{
    return charArray.at(nr);
}

short int SerialCommunication::getLastReadInt(unsigned char nr)
{
    return intArray.at(nr);
}

CommunicationError SerialCommunication::execute()
{
    unsigned char checksum = 0;
    unsigned char messageLenght = 0;

    checksum -= nodeNr;

    for (auto it = commandArray.begin(); it != commandArray.end(); ++it)
    {
        if (*it >= 128)
        {
            checksum -= *it;
            messageLenght += 1;
        }
        else if (*it >= 64)
        {
            checksum -= *it;
            ++it;
            checksum -= *it;
            ++it;
            checksum -= *it;
            messageLenght += 3;
        }
        else
        {
            checksum -= *it;
            ++it;
            checksum -= *it;
            messageLenght += 2;
        }
    }

    checksum -= messageLenght;

    serial.write(nodeNr);
    serial.write(checksum);
    serial.write(messageLenght);

    for (unsigned char byte : commandArray)
    {
        if (!serial.write(byte))
        {
            serial.sendWrittenData();
            serial.write(byte);
        }
    }
    serial.sendWrittenData();

    commandArray.clear();

    auto clearReceiveArrayAtExit = createCallAfterReturnHandler([this]()
        {
            this->receiveArray.clear();
        });

    unsigned char c = 0;
    bool error = false;
    for (auto it = receiveArray.begin(); it != receiveArray.end(); ++it)
    {
        error = !serial.read(c, timeout);
        if (error)
        {
            serial.read(c, timeout);
            return CommunicationError(nodeNr, CommunicationError::NO_RESPONSE);
        }

        if (*it == c)
        {
            if (*it >= 64)
            {
                error = !serial.read(c, timeout);
                if (error)
                {
                    serial.read(c, timeout);
                    return CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_1);
                }
                short value = c;

                error = !serial.read(c, timeout);
                if (error)
                {
                    serial.read(c, timeout);
                    return CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_2);
                }
                value += c * static_cast<unsigned short>(256);
                intArray.at(*it - 64) = value;
            }
            else
            {
                error = !serial.read(c, timeout);
                if (error)
                {
                    serial.read(c, timeout);
                    return CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_3);
                }
                charArray.at(*it) = c;
            }
        }
        else
        {
            while (true)
            {
                error = !serial.read(c, timeout);
                if (error)
                {
                    serial.read(c, timeout);
                    break;
                }
            }

            return CommunicationError(nodeNr, CommunicationError::UNEXPECTED_RESPONSE);
        }
    }
    error = !serial.read(c, timeout);
    if (error)
    {
        serial.read(c, timeout);
        return CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_4);
    }
    if (c != 0xff)
    {
        return CommunicationError(nodeNr, CommunicationError::CHECKSUM_ERROR);
    }

    return CommunicationError(nodeNr, CommunicationError::NO_ERROR);
}

DCServoCommunicator::DCServoCommunicator(unsigned char nodeNr, Communication* bus)
{
    activeIntReads.fill(true);
    activeCharReads.fill(true);
    this->nodeNr = nodeNr;
    this->bus = bus;

    communicationIsOk = false;
    initState = 0;
    backlashControlDisabled = false;
    newPositionReference = false;
    newOpenLoopControlSignal = false;

    setOffsetAndScaling(1.0f, 0);
}

void DCServoCommunicator::setOffsetAndScaling(float scale, float offset, float startPosition)
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

    long int wrapSize = (1l << 24) / positionUpscaling;

    if (pos - startPosition > wrapSize / 2)
    {
        offset -= wrapSize * scale;
    }
    else if (pos - startPosition < -wrapSize / 2)
    {
        offset += wrapSize * scale;
    }
}

void DCServoCommunicator::setControlSpeed(unsigned char controlSpeed)
{
    setControlSpeed(controlSpeed, controlSpeed * 4, controlSpeed * 32);
}

void DCServoCommunicator::setControlSpeed(unsigned char controlSpeed,
        unsigned short int velControlSpeed, unsigned short int filterSpeed)
{
    this->controlSpeed = controlSpeed;
    this->velControlSpeed = velControlSpeed;
    this->filterSpeed = filterSpeed;
}

void DCServoCommunicator::setBacklashControlSpeed(unsigned char backlashCompensationSpeed,
            float backlashCompensationCutOffSpeed,
            float backlashSize)
{
    this->backlashCompensationSpeed = backlashCompensationSpeed;
    this->backlashCompensationSpeedVelDecrease = static_cast<unsigned char>(stdmin::min(255.0f,
            255 * 10 / (backlashCompensationCutOffSpeed / stdmin::abs(scale))));
    this->backlashSize = static_cast<unsigned char>(backlashSize / stdmin::abs(scale));
}

void DCServoCommunicator::setFrictionCompensation(float fricComp)
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
        frictionCompensation = stdmin::abs(frictionCompensation);
    }
    else if (refVel < -4)
    {
        frictionCompensation = -stdmin::abs(frictionCompensation);
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

    return scale * (activeRefPos[2] * (1.0f / positionUpscaling) - pos);
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

float DCServoCommunicator::getScaling()
{
    return scale;
}

float DCServoCommunicator::getOffset()
{
    return offset;
}

CommunicationError DCServoCommunicator::run()
{
    bus->setNodeNr(nodeNr);

    for (size_t i = 0; i < activeIntReads.size(); i++)
    {
        if (activeIntReads[i])
        {
            bus->requestReadInt(i);
        }
    }

    for (size_t i = 0; i < activeCharReads.size(); i++)
    {
        if (activeCharReads[i])
        {
            bus->requestReadChar(i);
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
        bus->write(4, static_cast<char>(stdmin::round(velControlSpeed / 4.0f)));
        bus->write(5, static_cast<char>(stdmin::round(filterSpeed / 32.0f)));
        bus->write(6, static_cast<char>(backlashCompensationSpeed));
        bus->write(7, static_cast<char>(backlashCompensationSpeedVelDecrease));
        bus->write(8, static_cast<char>(backlashSize));
    }

    CommunicationError error = bus->execute();
    if (!error.noError())
    {
        return error;
    }

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

    for (size_t i = 0; i < activeCharReads.size(); i++)
    {
        if (activeCharReads[i])
        {

            if (isInitComplete())
            {
                activeCharReads[i] = false;
            }
            charReadBuffer[i] = bus->getLastReadChar(i);
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
        long int upscaledPos = static_cast<unsigned short int>(intReadBuffer[3]);
        upscaledPos += (static_cast<long int>(charReadBuffer[9]) << 16);
        if (upscaledPos >= (1l << 23))
        {
            upscaledPos -= (1l << 24);
        }

        long int encPosWithBacklashComp = intReadBuffer[10] + static_cast<long int>(intReadBuffer[11]);

        long int overflowedPart = ((upscaledPos - encPosWithBacklashComp) / (1l << 16)) << 16;

        intReadBufferIndex3Upscaling.set(upscaledPos);
        intReadBufferIndex10Upscaling.set(intReadBuffer[10]);
        intReadBufferIndex11Upscaling.set(intReadBuffer[11] + overflowedPart);
    }
    

    backlashEncoderPos = intReadBufferIndex3Upscaling.get() * (1.0f / positionUpscaling);
    encoderPos = intReadBufferIndex10Upscaling.get() * (1.0f / positionUpscaling);
    backlashCompensation = intReadBufferIndex11Upscaling.get() * (1.0f / positionUpscaling);

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

    return CommunicationError(nodeNr, CommunicationError::NO_ERROR);
}

#include "SerialComOptimizer.h"

SerialComOptimizer::SerialComOptimizer(Stream* serial, Stream* secSerial) :
    serial(serial),
    readBufferGetIt(readBuffer.end() - 1),
    readBufferPutIt(readBuffer.begin()),
    writeBufferPutIt(writeBuffer.begin())
{
    serialVec.push_back(serial);
    if (secSerial != nullptr)
    {
        serialVec.push_back(secSerial);
    }
    comLastActiveTimestamp = millis();
}

SerialComOptimizer::SerialComOptimizer(const SerialComOptimizer& in) :
    serial(in.serial),
    serialVec(in.serialVec),
    comLastActiveTimestamp(in.comLastActiveTimestamp),
    bridgeSerialVec(in.bridgeSerialVec),
    readBuffer(in.readBuffer),
    readBufferGetIt((in.readBufferGetIt - in.readBuffer.begin()) + readBuffer.begin()),
    readBufferPutIt((in.readBufferPutIt - in.readBuffer.begin()) + readBuffer.begin()),
    writeBuffer(in.writeBuffer),
    writeBufferPutIt((in.writeBufferPutIt - in.writeBuffer.begin()) + writeBuffer.begin())
{
}

SerialComOptimizer::~SerialComOptimizer()
{
}

void SerialComOptimizer::addBridge(Stream* bridge)
{
    bridgeSerialVec.push_back(bridge);
}

size_t SerialComOptimizer::available()
{
    int8_t bufferedAmount = readBufferPutIt - (readBufferGetIt + 1);
    if (bufferedAmount < 0)
    {
        bufferedAmount = readBuffer.size() + bufferedAmount;
    }
    return bufferedAmount;
}

uint8_t SerialComOptimizer::read()
{
    auto temp = readBufferGetIt;
    ++temp;
    if (temp == readBuffer.end())
    {
        temp = readBuffer.begin();
    }
    if (temp != readBufferPutIt)
    {
        readBufferGetIt = temp;
    }

    return *readBufferGetIt;
}

bool SerialComOptimizer::write(uint8_t byte)
{
    if (writeBufferPutIt == writeBuffer.end())
    {
        return false;
    }

    *writeBufferPutIt = byte;
    ++writeBufferPutIt;
    return true;
}

void SerialComOptimizer::collectReadData()
{
    int32_t readAmount = readBufferGetIt - readBufferPutIt;
    if (readAmount < 0)
    {
        readAmount = readBuffer.size() + readAmount;
    }

    if (readAmount == 0)
    {
        return;
    }

    int32_t availableInHardwarBuffer = serial->available();

    if (availableInHardwarBuffer != 0)
    {
        comLastActiveTimestamp = millis();
    }
    if (millis() - comLastActiveTimestamp > 1000)
    {
        for (auto s : serialVec)
        {
            availableInHardwarBuffer = s->available();

            if (availableInHardwarBuffer != 0)
            {
                serial = s;
                break;
            }
        }
    }

    if (availableInHardwarBuffer < readAmount)
    {
        readAmount = availableInHardwarBuffer;
    }

    std::array<char, 32> tempReadBuffer;
    readAmount = serial->readBytes(tempReadBuffer.data(), static_cast<size_t>(readAmount));

    if (readAmount < 0)
    {
        readAmount = 0;
    }

    repeatReceivedToBridges(tempReadBuffer, readAmount);

    for (auto it = tempReadBuffer.begin(); it != tempReadBuffer.begin() + readAmount; ++it)
    {
        *readBufferPutIt = *it;

        ++readBufferPutIt;
        if (readBufferPutIt == readBuffer.end())
        {
            readBufferPutIt = readBuffer.begin();
        }
    }
}

void SerialComOptimizer::sendWrittenData()
{
    writeBridgedResponse();

    serial->write(writeBuffer.data(), writeBufferPutIt - writeBuffer.begin());
    writeBufferPutIt = writeBuffer.begin();
}

void SerialComOptimizer::repeatReceivedToBridges(const std::array<char, 32>& buffer, int32_t messageLength)
{
    if (messageLength > 0)
    {
        for (auto s : bridgeSerialVec)
        {
            s->write(buffer.data(), messageLength);
        }
    }
}

void SerialComOptimizer::writeBridgedResponse()
{
    Stream* bridgeSerial = nullptr;
    int32_t availableToBridge = 0;
    for (auto s : bridgeSerialVec)
    {
        availableToBridge = s->available();

        if (availableToBridge != 0)
        {
            bridgeSerial = s;
            break;
        }
    }

    if (availableToBridge != 0)
    {
        std::array<char, 32> tempReadBuffer;
        int32_t sizeLeftInWriteBuffer = writeBuffer.end() - writeBufferPutIt;
        int32_t readAmount = tempReadBuffer.size();
        readAmount = std::min(readAmount, sizeLeftInWriteBuffer);
        readAmount = std::min(readAmount, availableToBridge);
        
        readAmount = bridgeSerial->readBytes(tempReadBuffer.data(), readAmount);

        for (int i = 0; i != readAmount; ++i)
        {
            this->write(tempReadBuffer[i]);
        }
    }
}

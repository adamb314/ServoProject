#include "ServoProject.h"

CommunicationError::CommunicationError(unsigned char nodeNr, ErrorCode code) :
        nodeNr(nodeNr), code(code)
{
    std::stringstream stringStream;
    stringStream << "Communication error: ";

    switch (code)
    {
        case COULD_NOT_SEND:
            stringStream << "Could not send to port";
            break;
        case NO_RESPONSE:
            stringStream << "No response from node " << static_cast<int>(nodeNr);
            break;
        case PARTIAL_RESPONSE_TYPE_1:
        case PARTIAL_RESPONSE_TYPE_2:
        case PARTIAL_RESPONSE_TYPE_3:
        case PARTIAL_RESPONSE_TYPE_4:
            stringStream << "Partial response from node " << static_cast<int>(nodeNr) << ", "
                    << "error code " << code;
            break;
        case UNEXPECTED_RESPONSE:
            stringStream << "Unexpected response from node " << static_cast<int>(nodeNr);
            break;
        case CHECKSUM_ERROR:
            stringStream << "Checksum error from node " << static_cast<int>(nodeNr);
            break;
    }
    whatString = stringStream.str();
}

const char* CommunicationError::what() const throw()
{
    try
    {
        return whatString.c_str();
    }
    catch (...)
    {
        return "Communication Error";
    }
}

SerialCommunication::SerialCommunication(std::string devName) :
        io(), port(io), reader(port, 50)
{
    nodeNr = 1;

    port.open(devName);
    port.set_option(boost::asio::serial_port_base::baud_rate(115200));
}

SerialCommunication::SerialCommunication() :
        io(), port(io), reader(port, 50)
{
    nodeNr = 1;
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

void SerialCommunication::execute()
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

    sendBuffer.clear();
    sendBuffer.push_back(nodeNr);
    sendBuffer.push_back(checksum);
    sendBuffer.push_back(messageLenght);
    sendBuffer.insert(sendBuffer.end(), commandArray.begin(), commandArray.end());

    size_t bytesSent = ::write(port.lowest_layer().native_handle(), &sendBuffer[0], sendBuffer.size());
    if (bytesSent != sendBuffer.size())
    {
        throw CommunicationError(nodeNr, CommunicationError::COULD_NOT_SEND);
    }

    commandArray.clear();

    std::vector<unsigned char> receiveArrayCopy = receiveArray;
    receiveArray.clear();

    char c = 0;
    bool error = false;
    for (auto it = receiveArrayCopy.begin(); it != receiveArrayCopy.end(); ++it)
    {
        error = !reader.read_char(c);
        if (error)
        {
            reader.read_char(c);
            throw CommunicationError(nodeNr, CommunicationError::NO_RESPONSE);
        }

        if (*it == c)
        {
            if (*it >= 64)
            {
                error = !reader.read_char(c);
                if (error)
                {
                    reader.read_char(c);
                    throw CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_1);
                }
                short value = static_cast<unsigned char>(c);

                error = !reader.read_char(c);
                if (error)
                {
                    reader.read_char(c);
                    throw CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_2);
                }
                value += static_cast<unsigned char>(c) * static_cast<unsigned short>(256);
                intArray.at(*it - 64) = value;
            }
            else
            {
                error = !reader.read_char(c);
                if (error)
                {
                    reader.read_char(c);
                    throw CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_3);
                }
                charArray.at(*it) = c;
            }
        }
        else
        {
            while (true)
            {
                error = !reader.read_char(c);
                if (error)
                {
                    reader.read_char(c);
                    break;
                }
            }

            throw CommunicationError(nodeNr, CommunicationError::UNEXPECTED_RESPONSE);
        }
    }
    error = !reader.read_char(c);
    if (error)
    {
        reader.read_char(c);
        throw CommunicationError(nodeNr, CommunicationError::PARTIAL_RESPONSE_TYPE_4);
    }
    if (static_cast<unsigned char>(c) != 0xff)
    {
        throw CommunicationError(nodeNr, CommunicationError::CHECKSUM_ERROR);
    }
}

void SerialCommunication::blocking_reader::read_complete(const boost::system::error_code& error,
                    size_t bytes_transferred)
{        

    read_error = (error || bytes_transferred == 0);
    
    // Read has finished, so cancel the
    // timer.
    timer.cancel();
}

void SerialCommunication::blocking_reader::time_out(const boost::system::error_code& error)
{

    // Was the timeout was cancelled?
    if (error)
    {
        // yes
        return;
    }

    // no, we have timed out, so kill
    // the read operation
    // The read callback will be called
    // with an error
    port.cancel();
}

SerialCommunication::blocking_reader::blocking_reader(boost::asio::serial_port& port, size_t timeout) :
                                            port(port), timeout(timeout),
                                            timer(port.get_executor()),
                                            read_error(true)
{
     
}

bool SerialCommunication::blocking_reader::read_char(char& val)
{
    
    val = c = '\0';

    // After a timeout & cancel it seems we need
    // to do a reset for subsequent reads to work.
    
    boost::asio::execution_context& e_context = port.get_executor().context();
    boost::asio::io_context& context_instance = static_cast<boost::asio::io_context&>(e_context);
    context_instance.reset();

    // Asynchronously read 1 character.
    boost::asio::async_read(port, boost::asio::buffer(&c, 1), 
            boost::bind(&blocking_reader::read_complete, 
                    this, 
                    boost::asio::placeholders::error, 
                    boost::asio::placeholders::bytes_transferred)); 

    // Setup a deadline time to implement our timeout.
    timer.expires_from_now(boost::posix_time::milliseconds(timeout));
    timer.async_wait(boost::bind(&blocking_reader::time_out,
                            this, boost::asio::placeholders::error));

    // This will block until a character is read
    // or until the it is cancelled.
    context_instance.run();

    if (!read_error)
        val = c;

    return !read_error;
}

void SimulateCommunication::execute()
{
    while (servoSims.size() < nodeNr)
    {
        servoSims.push_back(ServoSim());
    }

    auto& servo = servoSims.at(nodeNr - 1);
    servo.run();

    for (auto it = commandArray.begin(); it != commandArray.end(); ++it)
    {
        if (*it >= 128)
        {
            //read request, do nothing in sim
        }
        else if (*it >= 64)
        {
            const unsigned char intNr = *it - 64;
            ++it;
            short value = static_cast<unsigned char>(*it);
            ++it;
            value += static_cast<unsigned char>(*it) * static_cast<unsigned short>(256);
            servo.intArray.at(intNr) = value;
        }
        else
        {
            const unsigned char charNr = *it;
            ++it;
            servo.charArray.at(charNr) = static_cast<unsigned char>(*it);
        }
    }

    commandArray.clear();

    std::vector<unsigned char> receiveArrayCopy = receiveArray;
    receiveArray.clear();

    for (auto it = receiveArrayCopy.begin(); it != receiveArrayCopy.end(); ++it)
    {
        if (*it >= 64)
        {
            short value = servo.intArray.at(*it - 64);
            intArray.at(*it - 64) = value;
        }
        else
        {
            char value = servo.charArray.at(*it);
            charArray.at(*it) = value;
        }
    }
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

void DCServoCommunicator::setControlSpeed(unsigned char controlSpeed, double inertiaMarg)
{
    setControlSpeed(controlSpeed, controlSpeed * 4, controlSpeed * 32, inertiaMarg);
}

void DCServoCommunicator::setControlSpeed(unsigned char controlSpeed,
        unsigned short int velControlSpeed, unsigned short int filterSpeed,
        double inertiaMarg)
{
    this->controlSpeed = controlSpeed;
    this->velControlSpeed = velControlSpeed;
    this->filterSpeed = filterSpeed;
    inertiaMarg = std::min(std::max(inertiaMarg, 1.0), 1.0 + 255.0 / 128);
    this->inertiaMarg = static_cast<unsigned char>(std::round((inertiaMarg - 1.0) * 128.0));
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
    return initState >= 10 and remoteTimeHandler.isInitialized();
}

bool DCServoCommunicator::isCommunicationOk() const
{
    return communicationIsOk;
}

void DCServoCommunicator::setReference(const float& pos, const float& vel, const float& feedforwardU)
{
    newPositionReference = true;
    newOpenLoopControlSignal = false;
    refPos = std::round((pos - offset) / scale * positionUpscaling);

    int sign = 0;
    if (vel > 0.0f)
    {
        sign = 1;
    }
    else if (vel < 0.0f)
    {
        sign = -1;
    }
    refVel = std::round(vel / scale);
    refVel = std::max(1, std::abs(refVel)) * sign;

    if (refVel > 4)
    {
        frictionCompensation = std::abs(frictionCompensation);
    }
    else if (refVel < -4)
    {
        frictionCompensation = -std::abs(frictionCompensation);
    }
    this->feedforwardU = std::round(feedforwardU + frictionCompensation);
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

double DCServoCommunicator::getTime() const
{
    activeCharReads[11] = true;
    return remoteTimeHandler.get();
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

    for (size_t i = 0; i < activeCharReads.size(); i++)
    {
        if (activeCharReads[i])
        {
            bus->requestReadChar(i);
        }
    }

    bool loopNrReadActive = activeCharReads[11];

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
        bus->write(4, static_cast<char>(std::round(velControlSpeed / 4.0)));
        bus->write(5, static_cast<char>(std::round(filterSpeed / 32.0)));
        bus->write(10, static_cast<char>(inertiaMarg));
        bus->write(6, static_cast<char>(backlashCompensationSpeed));
        bus->write(7, static_cast<char>(backlashCompensationSpeedVelDecrease));
        bus->write(8, static_cast<char>(backlashSize));
    }

    bus->execute();

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

        if (loopNrReadActive)
        {
            remoteTimeHandler.update(charReadBuffer[11]);
        }
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

        remoteTimeHandler.initialize(charReadBuffer[11]);

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

DCServoCommunicator::ControlLoopSyncedTimeHandler::ControlLoopSyncedTimeHandler()
{
}

bool DCServoCommunicator::ControlLoopSyncedTimeHandler::isInitialized() const
{
    return loopCycleTime != 0.0;
}

bool DCServoCommunicator::ControlLoopSyncedTimeHandler::initialize(unsigned char loopNr)
{
    if (isInitialized())
    {
        return true;
    }

    if (initDataList.size() < 20)
    {
        initDataList.push_back(InitData{getLocalTime(), loopNr});
        return false;
    }

    std::vector<double> listOfDt;
    for (size_t i = 0; i != initDataList.size() - 1; ++i)
    {
        const auto& d0 = initDataList[i];
        const auto& d1 = initDataList[i + 1];
        double localTimeDiff = d1.localTime - d0.localTime;
        char loopNrDiff = static_cast<char>(d1.loopNr - d0.loopNr);

        if (loopNrDiff == 0)
        {
            continue;
        }

        listOfDt.push_back(localTimeDiff / loopNrDiff);
    }

    std::sort(listOfDt.begin(), listOfDt.end());

    size_t cutI = listOfDt.size() / 4;
    listOfDt = std::vector<double>{listOfDt.begin() + cutI, listOfDt.end() - cutI};

    if (listOfDt.size() == 0)
    {
        loopCycleTime = -1.0;
        return true;
    }

    loopCycleTime = 0.0;
    for (auto dt : listOfDt)
    {
        loopCycleTime += dt;
    }
    loopCycleTime /= listOfDt.size();
    loopCycleTime = std::round(loopCycleTime / us200) * us200;
    return true;
}

void DCServoCommunicator::ControlLoopSyncedTimeHandler::update(unsigned char loopNr)
{
    double localTime = getLocalTime();

    if (loopCycleTime == -1.0)
    {
        lastRemoteTime = localTime;
        return;
    }

    double localTimeDiff = localTime - initDataList.back().localTime;

    unsigned long nrOfLoops = static_cast<unsigned char>(loopNr - initDataList.back().loopNr);
    nrOfLoops += std::round((localTimeDiff / loopCycleTime - nrOfLoops) / 256) * 256;

    lastRemoteTime += nrOfLoops * loopCycleTime;

    initDataList.back().localTime = localTime;
    initDataList.back().loopNr = loopNr;
}

double DCServoCommunicator::ControlLoopSyncedTimeHandler::get() const
{
  return lastRemoteTime;
}

double DCServoCommunicator::ControlLoopSyncedTimeHandler::getLocalTime() const
{
    using namespace std::chrono;

    if (initDataList.size() == 0)
    {
        initTimePoint = high_resolution_clock::now();
        return 0.0;
    }

    double localTime = std::chrono::duration<double>(high_resolution_clock::now()
            - initTimePoint).count();
    return localTime;
}

ServoManager::ServoManager(double cycleTime,
        std::function<std::vector<std::unique_ptr<DCServoCommunicator> >() > initFunction,
        bool startManager) :
    servos{initFunction()},
    cycleTime{cycleTime}
{
    while (true)
    {
        bool allDone = true;
        for (auto& s : servos)
        {
            allDone &= s->isInitComplete();
            s->run();
        }

        if (allDone)
        {
            break;
        }
    }

    for (auto& s : servos)
    {
        currentPosition.push_back(s->getPosition());
    }

    if (startManager)
    {
        start();
    }
}

ServoManager::~ServoManager()
{
    shutdown();
}

void ServoManager::run()
{
    while (waitForThreadInit && !shuttingDown)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    using namespace std::chrono;
    high_resolution_clock::time_point sleepUntilTimePoint = high_resolution_clock::now();
    high_resolution_clock::duration clockDurationCycleTime(
            duration_cast<high_resolution_clock::duration>(duration<double>(cycleTime)));

    while (!shuttingDown)
    {
        try
        {
            cycleSleepTime = std::chrono::duration<double>(sleepUntilTimePoint - high_resolution_clock::now()).count();
            std::this_thread::sleep_until(sleepUntilTimePoint);
            sleepUntilTimePoint += clockDurationCycleTime;

            std::function<void(double, ServoManager&)> tempSendHandlerFunction;
            std::function<void(double, ServoManager&)> tempReadHandlerFunction;

            {
                const std::lock_guard<std::mutex> lock(handlerFunctionMutex);
                tempSendHandlerFunction = sendCommandHandlerFunction;
                tempReadHandlerFunction = readResultHandlerFunction;
            }

            if (tempSendHandlerFunction)
            {
                tempSendHandlerFunction(cycleTime, *this);
            }

            for (auto& s : servos)
            {
                s->run();
            }

            for (size_t i = 0; i != servos.size(); ++i)
            {
                currentPosition[i] = servos[i]->getPosition();
            }

            if (tempReadHandlerFunction)
            {
                tempReadHandlerFunction(cycleTime, *this);
            }
        }
        catch (...)
        {
            auto e = std::current_exception();

            std::function<void(std::exception_ptr e)> tempErrorHandlerFunction;

            {
                const std::lock_guard<std::mutex> lock(handlerFunctionMutex);
                tempErrorHandlerFunction = errorHandlerFunction;
            }

            if (tempErrorHandlerFunction)
            {
                shutdown();
                tempErrorHandlerFunction(e);
            }
            else if (delayedExceptionsEnabled)
            {
                registerUnhandledException(e);
                shutdown();
            }
            else
            {
                std::rethrow_exception(e);
            }
        }
    }
}

std::vector<double> ServoManager::getPosition() const
{
    return currentPosition;
}

void ServoManager::setHandlerFunctions(std::function<void(double, ServoManager&)> newSendCommandHandlerFunction, 
        std::function<void(double, ServoManager&)> newReadResultHandlerFunction,
        std::function<void(std::exception_ptr e)> newErrorHandlerFunction)
{
    const std::lock_guard<std::mutex> lock(handlerFunctionMutex);

    sendCommandHandlerFunction = newSendCommandHandlerFunction;
    readResultHandlerFunction = newReadResultHandlerFunction;
    errorHandlerFunction = newErrorHandlerFunction;
}

void ServoManager::removeHandlerFunctions()
{
    std::function<void(double, ServoManager&)> nullFun;
    setHandlerFunctions(nullFun, nullFun);
}

void ServoManager::start(std::function<void(std::thread&)> threadInitFunction)
{
    shuttingDown = false;
    if (t.get_id() == std::this_thread::get_id())
    {
        return;
    }

    if (!t.joinable())
    {
        waitForThreadInit = true;
        t = std::thread{&ServoManager::run, this};
        threadInitFunction(t);
        waitForThreadInit = false;
    }
}

void ServoManager::shutdown()
{
    shuttingDown = true;
    if (t.get_id() == std::this_thread::get_id())
    {
        return;
    }

    if (t.joinable())
    {
        t.join();
    }
}

void ServoManager::registerUnhandledException(std::exception_ptr e)
{
    const std::lock_guard<std::mutex> lock(handlerFunctionMutex);

    exception = e;
}

void ServoManager::enableDelayedExceptions(bool enable)
{
    delayedExceptionsEnabled = enable;
}

std::exception_ptr ServoManager::getUnhandledException()
{
    const std::lock_guard<std::mutex> lock(handlerFunctionMutex);

    auto e = exception;
    exception = std::exception_ptr();
    return e;
}

bool ServoManager::isAlive(bool raiseException)
{
    if (raiseException)
    {
        auto e = std::exception_ptr();
        if (shuttingDown)
        {
            e = getUnhandledException();
        }
        if (e)
        {
            std::rethrow_exception(e);
        }
    }
    return !shuttingDown;
}

double ServoManager::getCycleSleepTime() const
{
    return cycleSleepTime;
}

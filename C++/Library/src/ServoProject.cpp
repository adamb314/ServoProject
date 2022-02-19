#include "ServoProject.h"

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

bool SerialCommunication::execute()
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
        throw -1;
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
            std::cout << "error1\n";
            reader.read_char(c);
            return false;
        }

        if (*it == c)
        {
            if (*it >= 64)
            {
                error = !reader.read_char(c);
                if (error)
                {
                    std::cout << "error2\n";
                    reader.read_char(c);
                    return false;
                }
                short value = static_cast<unsigned char>(c);

                error = !reader.read_char(c);
                if (error)
                {
                    std::cout << "error3\n";
                    reader.read_char(c);
                    return false;
                }
                value += static_cast<unsigned char>(c) * static_cast<unsigned short>(256);
                intArray.at(*it - 64) = value;
            }
            else
            {
                error = !reader.read_char(c);
                if (error)
                {
                    std::cout << "error4\n";
                    reader.read_char(c);
                    return false;
                }
                charArray.at(*it) = c;
            }
        }
        else
        {
            std::cout << "error5\n";

            while (true)
            {
                error = !reader.read_char(c);
                if (error)
                {
                    reader.read_char(c);
                    break;
                }
            }
        }
    }
    error = !reader.read_char(c);
    if (error)
    {
        reader.read_char(c);
        return false;
    }
    if (static_cast<unsigned char>(c) != 0xff)
    {
        return false;
    }

    return true;
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

bool SimulateCommunication::execute()
{
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

    return true;
}

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
        bus->write(4, static_cast<char>(std::round(velControlSpeed / 4.0)));
        bus->write(5, static_cast<char>(std::round(filterSpeed / 32.0)));
        bus->write(6, static_cast<char>(backlashCompensationSpeed));
        bus->write(7, static_cast<char>(backlashCompensationSpeedVelDecrease));
        bus->write(8, static_cast<char>(backlashSize));
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

ServoManager::ServoManager(double cycleTime,
        std::function<std::vector<std::unique_ptr<DCServoCommunicator> >() > initFunction) :
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

    start();
}

ServoManager::~ServoManager()
{
    shutdown();
}

void ServoManager::run()
{
    try
    {
        using namespace std::chrono;
        high_resolution_clock::time_point sleepUntilTimePoint = high_resolution_clock::now();
        high_resolution_clock::duration clockDurationCycleTime(
                duration_cast<high_resolution_clock::duration>(duration<double>(cycleTime)));

        while (!shuttingDown)
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
    }
    catch (std::exception& e)
    {
        shuttingDown = true;

        const std::lock_guard<std::mutex> lock(handlerFunctionMutex);

        if (errorHandlerFunction)
        {
            errorHandlerFunction(e);
        }
        else
        {
            exception = std::make_unique<std::exception>(e);
            throw e;
        }
    }
}

std::vector<double> ServoManager::getPosition() const
{
    return currentPosition;
}

void ServoManager::setHandlerFunctions(std::function<void(double, ServoManager&)> newSendCommandHandlerFunction, 
        std::function<void(double, ServoManager&)> newReadResultHandlerFunction,
        std::function<void(std::exception& e)> newErrorHandlerFunction)
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

void ServoManager::start()
{
    if (shuttingDown)
    {
        shuttingDown = false;
        t = std::thread{&ServoManager::run, this};
    }
}

void ServoManager::shutdown()
{
    if (!shuttingDown)
    {
        shuttingDown = true;
        t.join();
    }
}

std::exception ServoManager::getUnhandledException()
{
    const std::lock_guard<std::mutex> lock(handlerFunctionMutex);
    
    auto e = *exception.get();
    exception.reset();

    return e;
}

bool ServoManager::isAlive(bool raiseException)
{
    if (raiseException)
    {
        const std::lock_guard<std::mutex> lock(handlerFunctionMutex);

        if (exception)
        {
            throw getUnhandledException();
        }
    }
    return !shuttingDown;
}

double ServoManager::getCycleSleepTime() const
{
    return cycleSleepTime;
}

//#include <type_traits>
#include <array>
#include <vector>
#include <exception>
#include <sstream>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp> 
#include <boost/bind/bind.hpp>

#ifndef SERVO_PROJECT_H
#define SERVO_PROJECT_H

class CommunicationError : public std::exception
{
public:
    enum ErrorCode
    {
        COULD_NOT_SEND = 1,
        NO_RESPONSE,
        PARTIAL_RESPONSE_TYPE_1,
        PARTIAL_RESPONSE_TYPE_2,
        PARTIAL_RESPONSE_TYPE_3,
        PARTIAL_RESPONSE_TYPE_4,
        UNEXPECTED_RESPONSE,
        CHECKSUM_ERROR
    };

    CommunicationError(unsigned char nodeNr, ErrorCode code);

    virtual ~CommunicationError() throw(){};

    virtual const char*
    what() const throw();

    std::string whatString;
    unsigned char nodeNr;
    ErrorCode code;
};

class Communication
{
public:
    Communication(){}
    ~Communication(){};

    virtual void setNodeNr(unsigned char nr) = 0;

    virtual void write(unsigned char nr, char value) = 0;

    virtual void write(unsigned char nr, short int value) = 0;

    virtual void requestReadChar(unsigned char nr) = 0;

    virtual void requestReadInt(unsigned char nr) = 0;

    virtual char getLastReadChar(unsigned char nr) = 0;

    virtual short int getLastReadInt(unsigned char nr) = 0;

    virtual void execute() = 0;
};

class SerialCommunication : public Communication
{
public:
    SerialCommunication(std::string devName);

protected:
    SerialCommunication();

public:
    virtual void setNodeNr(unsigned char nr);

    virtual void write(unsigned char nr, char value);

    virtual void write(unsigned char nr, short int value);

    virtual void requestReadChar(unsigned char nr);

    virtual void requestReadInt(unsigned char nr);

    virtual char getLastReadChar(unsigned char nr);

    virtual short int getLastReadInt(unsigned char nr);

    virtual void execute();

protected:
    class blocking_reader
    {
        boost::asio::serial_port& port;
        size_t timeout;
        char c;
        boost::asio::deadline_timer timer;
        bool read_error;
     
        // Called when an async read completes or has been cancelled
        void read_complete(const boost::system::error_code& error,
                            size_t bytes_transferred);
     
        // Called when the timer's deadline expires.
        void time_out(const boost::system::error_code& error);
     
    public:
     
        // Constructs a blocking reader, pass in an open serial_port and
        // a timeout in milliseconds.
        blocking_reader(boost::asio::serial_port& port, size_t timeout);
     
        // Reads a character or times out
        // returns false if the read times out
        bool read_char(char& val);
    };

    std::vector<unsigned char> commandArray;
    std::vector<unsigned char> receiveArray;

    std::vector<unsigned char> sendBuffer;

    unsigned char nodeNr;
    std::array<char, 16> charArray{0};
    std::array<short int, 16> intArray{0};

    boost::asio::io_service io;
    boost::asio::serial_port port;

    blocking_reader reader;
};

class SimulateCommunication : public SerialCommunication
{
public:
    SimulateCommunication()
    {
    }

    virtual void execute() override;

    class ServoSim
    {
    public:
        void run()
        {
            intArray.at(3) = intArray.at(0);
            intArray.at(10) = intArray.at(0);
            intArray.at(4) = intArray.at(1);
            intArray.at(5) = intArray.at(2);
        }

        std::array<char, 16> charArray{0};
        std::array<short int, 16> intArray{0};
    };

    std::vector<ServoSim> servoSims;
};

template <typename T, typename U>
class ContinuousValueUpCaster
{
  public:
    typedef typename std::decay<T>::type ValueType;
    typedef typename std::decay<U>::type InputType;

    const ValueType& get()
    {
        return value;
    }

    void set(const ValueType& v)
    {
        value = v;
    }

    void update(const InputType& input)
    {
        typedef typename std::make_signed<InputType>::type SignedInputType;

        SignedInputType diff = input - value;

        value += diff;
    }

  protected:
    ValueType value{0};
};

class DCServoCommunicator
{
  public:
    class OpticalEncoderChannelData
    {
    public:
        unsigned short int a{0};
        unsigned short int b{0};
        unsigned short int minCostIndex{0};
        unsigned short int minCost{0};
    };

    DCServoCommunicator(unsigned char nodeNr, Communication* bus);

    DCServoCommunicator(const DCServoCommunicator&) = delete;

    void setOffsetAndScaling(double scale, double offset, double startPosition = 0);

    void setControlSpeed(unsigned char controlSpeed);
    void setControlSpeed(unsigned char controlSpeed, unsigned short int velControlSpeed,
            unsigned short int filterSpeed);

    void setBacklashControlSpeed(unsigned char backlashCompensationSpeed,
            double backlashCompensationCutOffSpeed, double backlashSize);

    void setFrictionCompensation(double fricComp);

    void disableBacklashControl(bool b = true);

    bool isInitComplete() const;

    bool isCommunicationOk() const;

    void setReference(const float& pos, const float& vel, const float& feedforwardU);

    void setOpenLoopControlSignal(const float& feedforwardU, bool pwmMode);

    float getPosition(bool withBacklash = true) const;

    float getVelocity() const;

    float getControlSignal() const;

    float getFeedforwardU() const;

    float getCurrent() const;

    short int getPwmControlSignal() const;

    float getControlError(bool withBacklash = true) const;

    short int getCpuLoad() const;

    short int getLoopTime() const;

    float getBacklashCompensation() const;

    OpticalEncoderChannelData getOpticalEncoderChannelData() const;

    double getScaling();

    double getOffset();

    void run();

  private:
    void updateOffset();

    Communication* bus{nullptr};
    unsigned char nodeNr{0};

    bool communicationIsOk{false};

    int initState{0};
    bool backlashControlDisabled{false};
    bool newPositionReference{false};
    bool newOpenLoopControlSignal{false};
    bool pwmOpenLoopMode{false};

    unsigned char controlSpeed{50};
    unsigned short int velControlSpeed{50 * 4};
    unsigned short int filterSpeed{50 * 32};
    unsigned char backlashCompensationSpeed{10};
    unsigned char backlashCompensationSpeedVelDecrease{0};
    unsigned char backlashSize{0};

    mutable std::array<bool, 16> activeIntReads{false};
    std::array<short int, 16> intReadBuffer{0};

    mutable std::array<bool, 16> activeCharReads{false};
    std::array<char, 16> charReadBuffer{0};

    ContinuousValueUpCaster<long int, short int> intReadBufferIndex3Upscaling;
    ContinuousValueUpCaster<long int, short int> intReadBufferIndex10Upscaling;
    ContinuousValueUpCaster<long int, short int> intReadBufferIndex11Upscaling;

    float backlashEncoderPos{0.0};
    float encoderPos{0.0};
    float backlashCompensation{0.0};
    short int encoderVel{0};
    short int controlSignal{0};
    short int current{0};
    short int pwmControlSignal{0};
    short int cpuLoad{0};
    short int loopTime{0};
    OpticalEncoderChannelData opticalEncoderChannelData;

    long int refPos{0};
    std::array<long int, 5> activeRefPos{0};
    short int refVel{0};
    short int feedforwardU{0};
    std::array<short int, 5> activeFeedforwardU{0};
    double frictionCompensation{0.0};

    double offset{0.0};
    double startPosition{0.0};
    double scale{1.0};

    static constexpr int positionUpscaling = 32;
};

#include <chrono>
#include <thread>
#include <mutex>
#include <algorithm>
#include <functional>

class ServoManager
{
public:
    ServoManager(double cycleTime,
            std::function<std::vector<std::unique_ptr<DCServoCommunicator> >() > initFunction,
            bool startManager = true);

    virtual ~ServoManager();

    void run();

    std::vector<double> getPosition() const;

    void setHandlerFunctions(std::function<void(double, ServoManager&)> newSendCommandHandlerFunction, 
            std::function<void(double, ServoManager&)> newReadResultHandlerFunction,
            std::function<void(std::exception_ptr e)> newErrorHandlerFunction = std::function<void(std::exception_ptr e)>());

    void removeHandlerFunctions();

    void start(std::function<void(std::thread&)> threadInitFunction = [](std::thread& t){});

    void shutdown();

    void enableDelayedExceptions(bool enable = true);

    std::exception_ptr getUnhandledException();

    bool isAlive(bool raiseException = true);

    double getCycleSleepTime() const;

    std::vector<std::unique_ptr<DCServoCommunicator> > servos;

protected:

    void registerUnhandledException(std::exception_ptr e);

    std::vector<double> currentPosition;

    double cycleTime;
    double cycleSleepTime{0.0};
    bool shuttingDown{true};
    bool waitForThreadInit{true};
    bool delayedExceptionsEnabled{false};
    std::exception_ptr exception;

    std::thread t;
    std::mutex handlerFunctionMutex;
    std::function<void(double, ServoManager&)> sendCommandHandlerFunction;
    std::function<void(double, ServoManager&)> readResultHandlerFunction;
    std::function<void(std::exception_ptr)> errorHandlerFunction;
};

#endif

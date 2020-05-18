#include <iostream>
#include <fstream>
#include <exception>
#include <array>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp> 
#include <boost/bind.hpp>

#ifndef MASTER_COMMUNICATION_H
#define MASTER_COMMUNICATION_H

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

    virtual bool execute() = 0;
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

    virtual bool execute();

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
    std::array<char, 8> charArray{0};
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

    virtual bool execute() override;

    class ServoSim
    {
    public:
        void run()
        {
            intArray.at(3) = intArray.at(0);
            intArray.at(9) = intArray.at(0);
            intArray.at(4) = intArray.at(1);
            intArray.at(5) = intArray.at(2);
        }

        std::array<char, 8> charArray{0};
        std::array<short int, 16> intArray{0};
    };

    std::array<ServoSim, 6> servoSims{};
};

#endif
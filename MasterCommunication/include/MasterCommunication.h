#include <iostream>
#include <fstream>
#include <exception>

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp> 
#include <boost/bind.hpp>

#ifndef MASTER_COMMUNICATION_H
#define MASTER_COMMUNICATION_H


class Communication
{
  public:
    Communication(std::string devName);

    void setNodeNr(unsigned char nr);

    void write(unsigned char nr, char value);

    void write(unsigned char nr, int value);

    void requestReadChar(unsigned char nr);

    void requestReadInt(unsigned char nr);

    char getLastReadChar(unsigned char nr);

    int getLastReadInt(unsigned char nr);

    bool execute();

  private:
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
    char charArray[8];
    int intArray[16];

    boost::asio::io_service io;
    boost::asio::serial_port port;

    blocking_reader reader;
};

#endif
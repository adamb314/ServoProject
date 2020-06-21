#include "MasterCommunication.h"

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

    std::this_thread::sleep_for(std::chrono::milliseconds(3));

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

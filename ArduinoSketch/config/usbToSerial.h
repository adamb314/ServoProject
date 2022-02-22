#ifndef CONFIG_HOLDER_H
#define CONFIG_HOLDER_H

#include "../defaultConfigHolder.h"

class ConfigHolder
{
public:
    static std::unique_ptr<Communication> getCommunicationHandler()
    {
        Serial.begin(115200);
        Serial1.begin(115200);
        auto serialComOptimizer = SerialComOptimizer(&Serial);
        serialComOptimizer.addBridge(&Serial1);
        auto com = std::make_unique<Communication>(serialComOptimizer);

        // Optional.
        // By adding ServoCommunicationHandler nodes here you can control regular servos 
        // in the same way as if they were fully modified ServoProject servos.

        //com->addCommunicationNode(
        //        std::make_unique<ServoCommunicationHandler>(nodeNr, servoPin));

        return com;
    }
};

#endif

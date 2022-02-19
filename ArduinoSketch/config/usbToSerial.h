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

        com->addCommunicationNode(
                std::make_unique<ServoCommunicationHandler>(2, 7));

        return com;
    }
};

#endif

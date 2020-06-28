#include "defaultConfigHolder.h"

#ifndef CONFIG_HOLDER_H
#define CONFIG_HOLDER_H

class SetupConfigHolder : public DefaultConfigHolder
{
public:
    static constexpr float getMainEncoderGearRation()
    {
        return 275.0 / 125904.0;
    }

    static std::unique_ptr<Communication> getCommunicationHandler()
    {
        auto com = std::make_unique<Communication>(&Serial1, 115200);
        com->addCommunicationNode(std::make_unique<DCServoCommunicationHandler>(1));

        return com;
    }
};

class ConfigHolder : public SetupConfigHolder
{
public:
    class ControlParameters : public SetupConfigHolder::DefaultControlParameters
    {
      public:
    };
};
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#endif

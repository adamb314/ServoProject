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
        com->addCommunicationNode(std::make_unique<DCServoCommunicationHandler>(3));
        com->addCommunicationNode(std::make_unique<ServoCommunicationHandler>(4, 5));
        com->addCommunicationNode(std::make_unique<ServoCommunicationHandler>(5, 7));
        com->addCommunicationNode(std::make_unique<ServoCommunicationHandler>(6, 9));
        com->addCommunicationNode(std::make_unique<ServoCommunicationHandler>(7, 10));

        return com;
    }
};

// replace with generated output from systemIdent.py
// vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
// config setps
// 1) disconnect motor from gearbox
// 2) compile and transfer to servo nr x
// 3) open MasterCommunication folder in terminal
// 4) run 'make'
// 5) run './executable --servoNr x --recOpticalEncoder --output=opticalEncoderData.txt'
// 6) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt'
// 7) copy past new generated ConfigHolder class, from terminal, over old class
// 8) compile and transfer to servo nr x with new ConfigHolder class
// 9) run './executable --servoNr x --recSystemIdentData --output=systemIdentData.txt'
// 10) run './systemIdent.py --opticalEncoderDataFile=opticalEncoderData.txt --systemIdentDataFile=systemIdentData.txt'
// 11) copy past new generated ConfigHolder class, from terminal, over old class
// 12) connect motor to gearbox again
// 13) compile and transfer to servo nr x with new ConfigHolder class
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

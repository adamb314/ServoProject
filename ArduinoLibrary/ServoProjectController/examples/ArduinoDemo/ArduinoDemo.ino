#include <Arduino.h>
#include "ServoProjectController.h"

#define SIMULATE

#define DEBUG_SERIAL Serial

#ifndef SIMULATE
#define SERVO_BUS_SERIAL Serial1
#endif

constexpr size_t dof = 2;

constexpr int cycleTimeInMs = 18;

constexpr size_t maxNrOfTrajectoryItems = 21;

#ifdef SERVO_BUS_SERIAL
class ServoManager : public ServoManagerBase<dof>
{
public:
    ServoManager() : 
            ServoManagerBase<dof>(::cycleTimeInMs, &servos),
            servos{{{1, &com}, {2, &com}}}
    {
        servos[0].setOffsetAndScaling(360.0f / 4096.0f, -110.0f, 0);
        servos[0].setControlSpeed(32);
        servos[0].setBacklashControlSpeed(6, 180.0f, 0.0f);

        servos[1].setOffsetAndScaling(180.0f / 1900.0f, 0.0f, 0.0f);
    }

    CppArray<DCServoCommunicator, dof> servos;

private:
    SerialComOptimizer comOptimizer{&SERVO_BUS_SERIAL};
    SerialCommunication com{comOptimizer};
};
#endif

#ifdef SIMULATE
class SimulatedServoManager : public ServoManagerBase<dof>
{
public:
    SimulatedServoManager() : 
            ServoManagerBase<dof>(::cycleTimeInMs, &servos),
            servos{{{1, &simCom}, {2, &simCom}}}
    {
    }

    CppArray<DCServoCommunicator, dof> servos;

private:
    SimulateCommunication<dof> simCom;
};
#endif

void setup()
{
    delay(2000);

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.begin(115200);
#endif

#ifdef SERVO_BUS_SERIAL
    SERVO_BUS_SERIAL.begin(115200);
#endif
}

int skippPrintCounter = 0;
int state = 0;

void loop()
{
#ifdef SIMULATE
    static SimulatedServoManager servoManager;
#else
    static ServoManager servoManager;
#endif
    static TrajectoryBuilder<dof, maxNrOfTrajectoryItems> trajectoryBuilder(servoManager.getDt());
    static ServoReferenceSetter<dof> servoReferenceSetter(servoManager.getDt());

    CppArray<float, dof> position;
    bool added;

    switch (state)
    {
    case 0:
        servoManager.init();

        trajectoryBuilder.clear();

        position = servoManager.getPosition();
        trajectoryBuilder.setStart(position);

        trajectoryBuilder.addSmoothMove({{0.0f, 0.0f}}, 1.0f);

        trajectoryBuilder.addWait(10.0f);

        for (size_t i = 0; i != 3; ++i)
        {
            //trajectoryBuilder.addSmoothMove({{2.0f, 2.0f}}, 0.2f);
            trajectoryBuilder.addSmoothMove({{-10.0f, -10.0f}}, 1.0f);
            trajectoryBuilder.addSmoothMove({{0.0f, 0.0f}}, 3.0f);
            trajectoryBuilder.addWait(3.0f);
            //trajectoryBuilder.addSmoothMove({{-2.0f, -2.0f}}, 0.2f);
            trajectoryBuilder.addSmoothMove({{10.0f, 10.0f}}, 1.0f);
            trajectoryBuilder.addSmoothMove({{0.0f, 0.0f}}, 3.0f);
            trajectoryBuilder.addWait(3.0f);
        }

        added = trajectoryBuilder.addWait(7.0f);

        if (!added)
        {
#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.println("maxNrOfTrajectoryItems exceeded");
#endif
            // if last trajectory item exceeds maxNrOfTrajectoryItems
            // go to error state
            state = 1000;
        }
        else
        {
            state = 10;
        }
        break;

    case 10:
        position = trajectoryBuilder.get();
        servoReferenceSetter.set(servoManager.servos, position);

        if (!trajectoryBuilder.step())
        {
            state = 20;
        }

        position = servoManager.getPosition();

#ifdef DEBUG_SERIAL
        if (skippPrintCounter == 10)
        {
            skippPrintCounter = 0;
            DEBUG_SERIAL.print(position[0]);
            DEBUG_SERIAL.print(", ");
            DEBUG_SERIAL.println(position[1]);
        }
        ++skippPrintCounter;
#endif
        break;

    case 20:
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Done");
#endif
        state = 100;
        break;

    case 100:
        break;

    case 1000:
        return;
    }

    CommunicationError error = servoManager.run();
    if (!error.noError())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("Error on nodeNr ");
        DEBUG_SERIAL.print(error.nodeNr);
        DEBUG_SERIAL.print(" code ");
        DEBUG_SERIAL.println(error.code);
#endif
        state = 1000;
    }
}

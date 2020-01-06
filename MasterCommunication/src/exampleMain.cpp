#include <iostream>
#include <fstream>
#include <exception>

#include "MasterCommunication.h"
#include "DCServoCommunicator.h"

#include <time.h>

#include <cmath>
#include <Eigen/Dense>

const double pi = 3.1415926535897932384626433832795028841972;

#include <chrono>
#include <thread>

int main ()
{
    Communication communication("/dev/ttyACM0");
    DCServoCommunicator dcServo1(1, &communication);
    DCServoCommunicator dcServo2(2, &communication);
    DCServoCommunicator dcServo3(3, &communication);

    std::cout << "init start\n";

    while (!dcServo1.isInitComplete()
           || !dcServo2.isInitComplete()
           || !dcServo3.isInitComplete()
        )
    {
        dcServo1.run();
        dcServo2.run();
        dcServo3.run();
    }

    std::cout << "init done\n";

    dcServo1.setOffsetAndScaling(4096, 0);
    dcServo2.setOffsetAndScaling(4096, 0);
    dcServo3.setOffsetAndScaling(4096, 0);

    Eigen::Matrix<double, 3, 1> startPos;
    Eigen::Matrix<double, 3, 1> refPos;
    Eigen::Matrix<double, 3, 1> endPos;

    startPos[0] = dcServo1.getPosition();
    startPos[1] = dcServo2.getPosition();
    startPos[2] = dcServo3.getPosition();

    endPos[0] = 2048;
    endPos[1] = 2048;
    endPos[2] = 2048;

    double t = 0;
    double interpolT = 0;

    timespec lastEndOfLoopTimePoint;
    clock_gettime(CLOCK_MONOTONIC, &lastEndOfLoopTimePoint);

    while(t < 120)
    {
        double s = 0;
        if (t < 30)
        {
            s = (((static_cast<int>(t) % 5) / 3) - 0.5);
        }

        refPos = s * endPos + (1 - s) * startPos;

        dcServo1.setReference(refPos[0], 0, 0);
        dcServo2.setReference(refPos[1], 0, 0);
        dcServo3.setReference(refPos[2], 0, 0);

        dcServo1.run();
        dcServo2.run();
        dcServo3.run();

        timespec currentTimePoint;
        clock_gettime(CLOCK_MONOTONIC, &currentTimePoint);

        double cycleTime = timeDiff(currentTimePoint, lastEndOfLoopTimePoint);
        lastEndOfLoopTimePoint = currentTimePoint;

        std::cout << "t:" << t << " p0:" << dcServo1.getPosition() << " p1:" << dcServo2.getPosition() << " p2:" << dcServo3.getPosition()
                                   << " e0:" << dcServo1.getControlError() << " e1:" << dcServo2.getControlError() << " e2:" << dcServo3.getControlError()
                                   << " v0:" << dcServo1.getVelocity() << " v1:" << dcServo2.getVelocity() << " v2:" << dcServo3.getVelocity()
                                   << " c0:" << dcServo1.getControlSignal() << " c1:" << dcServo2.getControlSignal() << " c2:" << dcServo3.getControlSignal()
                                   << "\n";

        t += cycleTime;
    }

    return 0;
}

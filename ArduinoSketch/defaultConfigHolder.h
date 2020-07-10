#include <Eigen30.h>
#include "EncoderHandler.h"
#include "CurrentControlLoop.h"
#include "OpticalEncoderHandler.h"
#include "ArduinoC++BugFixes.h"
#include "CommunicationHandlers.h"

#ifndef DEFAULT_CONFIG_HOLDER_H
#define DEFAULT_CONFIG_HOLDER_H

class DefaultConfigHolder
{
public:
    static std::unique_ptr<CurrentController> createCurrentController()
    {
        return std::make_unique<CurrentControlLoop>(400);
    }

    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()
    {
        return std::make_unique<EncoderHandler>(A5);
    }

    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()
    {
        std::array<uint16_t, 512> aVec = {};
        std::array<uint16_t, 512> bVec = {};
        return std::make_unique<OpticalEncoderHandler>(aVec, bVec);
    }

    class DefaultControlParameters
    {
      public:
        static Eigen::Vector3f getKVector()
        {
            Eigen::Vector3f K;
            K << 0.0,
                0.0,
                0.0;

            return K;
        }

        static Eigen::Matrix3f getAMatrix()
        {
            Eigen::Matrix3f A;
            A << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;

            return A;
        }

        static Eigen::Matrix3f getAInvMatrix()
        {
            Eigen::Matrix3f AInv;
            AInv << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;

            return AInv;
        }

        static Eigen::Vector3f getBVector()
        {
            Eigen::Vector3f B;
            B << 0.0,
                0.0,
                0.0;

            return B;
        }

        static Eigen::Matrix<float, 4, 1> calculateLVector(uint8_t controllerSpeed,
                float dt, float a, float b)
        {
            float posControlPole = exp(-dt * controllerSpeed);
            float velControlPole[] = {exp(-1.0 * dt * 4 * controllerSpeed), exp(-0.9 * dt * 4 * controllerSpeed)};
    
            Eigen::Matrix<float, 4, 1> L;
            L[0] = (1.0 - posControlPole) / dt;
            L[1] = (a + 1 - velControlPole[0] - velControlPole[1]) / b;
            L[2] = (a - b * L[1] - velControlPole[0] * velControlPole[1]) / b;
            L[3] = 10 * L[2];

            return L;
        }

        static Eigen::Matrix<float, 4, 1> getLVector(uint8_t controllerSpeed)
        {
            float dt = getAMatrix()(0, 1);
            float a = getAMatrix()(1, 1);
            float b = getBVector()(1);

            return calculateLVector(controllerSpeed, dt, a, b);
        }
    };
};

#endif

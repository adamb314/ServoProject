#include <ArduinoEigenDense.h>
#include "src/Hardware/EncoderHandler.h"
#include "src/Control/CurrentControlLoop.h"
#include "src/Hardware/OpticalEncoderHandler.h"
#include "src/Hardware/ResistiveEncoderHandler.h"
#include "src/Hardware/SimulationHandler.h"
#include "src/ArduinoC++BugFixes.h"
#include "src/Communication/CommunicationHandlers.h"

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
        std::array<uint16_t, 2048> aVec = {};
        std::array<uint16_t, 2048> bVec = {};
        return std::make_unique<OpticalEncoderHandler>(aVec, bVec, A2, A3, 4096.0f);
    }

    template<size_t vecSize>
    static uint16_t pwmHighFrqCompFun(const std::array<uint16_t, vecSize>& linearizeVec, uint16_t in)
    {
        constexpr static uint16_t maxPwm = 1023;

        int32_t t = in * (vecSize - 1);
        size_t index = std::min(static_cast<size_t>(vecSize - 2),
                                static_cast<size_t>(t / maxPwm));
        t -= index * maxPwm;

        const uint16_t& a = linearizeVec[index];
        const uint16_t& b = linearizeVec[index + 1];

        return static_cast<uint16_t>((a * (maxPwm - t) + b * t + maxPwm / 2) / maxPwm);
    }

    class DefaultControlParameters
    {
      public:
        static Eigen::Vector3f getKVector()
        {
            Eigen::Vector3f K;
            K << 0.0f,
                0.0f,
                0.0f;

            return K;
        }

        static Eigen::Matrix3f getAMatrix()
        {
            Eigen::Matrix3f A;
            A << 1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f;

            return A;
        }

        static Eigen::Matrix3f getAInvMatrix()
        {
            Eigen::Matrix3f AInv;
            AInv << 1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f,
                0.0f, 0.0f, 1.0f;

            return AInv;
        }

        static Eigen::Vector3f getBVector()
        {
            Eigen::Vector3f B;
            B << 0.0f,
                0.0f,
                0.0f;

            return B;
        }

        static bool internalFeedForwardEnabled()
        {
            return false;
        }

        static float getMaxVelocity()
        {
            return std::numeric_limits<float>::max();
        }

        static float getFrictionComp()
        {
            return 0.0f;
        }

        static std::array<int16_t, 512> getPosDepForceCompVec()
        {
            std::array<int16_t, 512> posDepForceCompVec{0};

            return posDepForceCompVec;
        }

        static std::array<int16_t, 512> getPosDepFrictionCompVec()
        {
            std::array<int16_t, 512> posDepFrictionCompVec{0};

            return posDepFrictionCompVec;
        }
    };
};

#endif

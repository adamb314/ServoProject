#include <ArduinoEigenDense.h>
#include "EncoderHandler.h"
#include "CurrentControlLoop.h"
#include "OpticalEncoderHandler.h"
#include "ResistiveEncoderHandler.h"
#include "SimulationHandler.h"
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
    };
};

template<typename T>
std::unique_ptr<DCServo> createDCServo(uint8_t controlSpeed = 0, uint8_t backlashControlSpeed = 0)
{
    auto currentController = T::createCurrentController();
    auto mainEncoder = T::createMainEncoderHandler();
    auto outputEncoder = T::createOutputEncoderHandler();
    auto controlConfig = DefaultControlConfiguration::create<typename T::ControlParameters>(mainEncoder.get());
    bool enableInternalFeedForward = T::ControlParameters::internalFeedForwardEnabled();
    auto kalmanFilter = KalmanFilter::create<typename T::ControlParameters>();

    auto dcServo = std::make_unique<DCServo>(
            std::move(currentController),
            std::move(mainEncoder),
            std::move(outputEncoder),
            std::move(kalmanFilter),
            std::move(controlConfig));

    if (controlSpeed != 0)
    {
        dcServo->setControlSpeed(controlSpeed);
        dcServo->setBacklashControlSpeed(backlashControlSpeed);
    }

    if (enableInternalFeedForward)
    {
        dcServo->enableInternalFeedForward();
    }

    return dcServo;
}

#endif

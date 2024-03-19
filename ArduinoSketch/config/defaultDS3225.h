#ifndef CONFIG_HOLDER_H
#define CONFIG_HOLDER_H

#include "../defaultConfigHolder.h"

class SetupConfigHolder : public DefaultConfigHolder
{
public:
    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()
    {
        constexpr static std::array<uint16_t, 2048> aVec = {0};
        constexpr static std::array<uint16_t, 2048> bVec = {0};
        
        return std::make_unique<OpticalEncoderHandler>(aVec, bVec, A3, A4, 4096.0f * 10.0f / 1 * 11.0f / 62 * 14.0f / 48 * 13.0f / 45 * 1.0f / 42);
    }

    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()
    {
        constexpr static std::array<int16_t, 1025> compVec = {0};
        return std::make_unique<ResistiveEncoderHandler>(A1, 4096.0f * 220.0f / 360, compVec);
    }

    static std::unique_ptr<CurrentController> createCurrentController()
    {
        constexpr float pwmToStallCurrent{1.0f};
        constexpr float backEmfCurrent{0.0f};

        auto pwmHighFrqCompFun = [](uint16_t in)
        {
            return in;
        };
        auto pwm = std::make_unique<HBridgeHighResPin11And12Pwm>(true, true, pwmHighFrqCompFun, 24000);
        return std::make_unique<CurrentControlModel>(pwmToStallCurrent, backEmfCurrent, std::move(pwm));
    }

    class DefaultControlParametersWithPositionCompensation : public SetupConfigHolder::DefaultControlParameters
    {
        public:
        static std::array<int16_t, 512> getPosDepForceCompVec()
        {
            constexpr static std::array<int16_t, 512> vec = {0};
            return vec;
        }

        static std::array<int16_t, 512> getPosDepFrictionCompVec()
        {
            constexpr static std::array<int16_t, 512> vec = {0};
            return vec;
        }
    };

    class ControlParameters : public DefaultControlParametersWithPositionCompensation
    {
      public:
        //kalman filter observer vector
        static Eigen::Matrix<float, 3, 7> getKVector()
        {
            Eigen::Matrix<float, 3, 7> K;
            K << -1.9619302890043613e-23f, 6.763936428834821e-19f, -1.0211439280426281e-14f, 8.966380809069127e-11f, -5.026553398907519e-07f, 0.0017546745669688208f, -0.01965617492779689f,
                2.5494778082386736e-19f, -7.113271986018928e-15f, 7.773198181849748e-11f, -4.0726853722124173e-07f, 0.0008984664748649702f, 0.3759747733999441f, -61.62201356939168f,
                5.093177792677117e-19f, -1.8966263561104195e-14f, 2.783998596483012e-10f, -1.9943485906824697e-06f, 0.006503382629253611f, -2.7471624009629547f, 339.77668409510306f;

            return K;
        }

        //system model A matrix
        static Eigen::Matrix3f getAMatrix()
        {
            Eigen::Matrix3f A;
            A << 1.0f, 0.0005936462787358029f, 1.887871435352648e-05f,
                0.0f, 0.9788959531583871f, 0.06270613007078364f,
                0.0f, 0.0f, 1.0f;

            return A;
        }

        //system model invers A matrix
        static Eigen::Matrix3f getAInvMatrix()
        {
            Eigen::Matrix3f AInv;
            AInv << 1.0f, -0.0006064447164383669f, 1.9149086916197254e-05f,
                0.0f, 1.021559029612413f, -0.0640580133858595f,
                0.0f, 0.0f, 1.0f;

            return AInv;
        }

        //system model B matrix
        static Eigen::Vector3f getBVector()
        {
            Eigen::Vector3f B;
            B << 1.887871435352648e-05f,
                0.06270613007078364f,
                0.0f;

            return B;
        }

        static bool internalFeedForwardEnabled()
        {
            return true;
        }

        //system model friction comp value
        static float getFrictionComp()
        {
            return 58.36825891129791f;
        }
    };
};

class ConfigHolder
{
public:
    static std::unique_ptr<Communication> getCommunicationHandler()
    {
        Serial.begin(115200);
        Serial1.begin(115200);
        auto com = std::make_unique<Communication>(SerialComOptimizer(&Serial1, &Serial));
        com->addCommunicationNode(
                std::make_unique<DCServoCommunicationHandler>(1, createDCServo<SetupConfigHolder>()));

        return com;
    }
};

#endif

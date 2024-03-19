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
        
        return std::make_unique<OpticalEncoderHandler>(aVec, bVec, A3, A4, 4096.0f * 10.0f / 48.0 * 10.0f / 38.0 * 10.0f / 38.0 * 10.0f / 38.0);
    }

    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()
    {
        constexpr static std::array<int16_t, 1025> compVec = {0};
        return std::make_unique<ResistiveEncoderHandler>(A1, 4096.0f * 200.0f / 360, compVec);
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
            K << -1.9622364197173024e-23f, 6.764613340485041e-19f, -1.0211998919971856e-14f, 8.96659593729972e-11f, -5.026592018080586e-07f, 0.001754677360416462f, -0.006887646928972721f,
                2.5291289597823245e-19f, -7.054107394054792e-15f, 7.703046508627754e-11f, -4.0285856466740963e-07f, 0.0008823680645336847f, 0.4108714140087351f, -61.801326156486226f,
                1.3052175124930118e-19f, -4.860751271226963e-15f, 7.135234239869617e-11f, -5.111528899101177e-07f, 0.0016668424014543767f, -0.704100787886532f, 87.08405835980246f;

            return K;
        }

        //system model A matrix
        static Eigen::Matrix3f getAMatrix()
        {
            Eigen::Matrix3f A;
            A << 1.0f, 0.0005974960091957293f, 7.349786814334303e-05f,
                0.0f, 0.9916649912249555f, 0.2446516076963969f,
                0.0f, 0.0f, 1.0f;

            return A;
        }

        //system model invers A matrix
        static Eigen::Matrix3f getAInvMatrix()
        {
            Eigen::Matrix3f AInv;
            AInv << 1.0f, -0.0006025180020297697f, 7.390912971926106e-05f,
                0.0f, 1.0084050650661254f, -0.2467079203776173f,
                0.0f, 0.0f, 1.0f;

            return AInv;
        }

        //system model B matrix
        static Eigen::Vector3f getBVector()
        {
            Eigen::Vector3f B;
            B << 7.349786814334303e-05f,
                0.2446516076963969f,
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
            return 39.875460580407044f;
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

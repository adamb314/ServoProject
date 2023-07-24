#ifndef CONFIG_HOLDER_H
#define CONFIG_HOLDER_H

#include "../defaultConfigHolder.h"

class SetupConfigHolder : public DefaultConfigHolder
{
public:
    static SimulationHandler simHandler;

    static std::unique_ptr<OpticalEncoderHandler> createMainEncoderHandler()
    {
        constexpr static std::array<uint16_t, 2048> aVec = {0};
        constexpr static std::array<uint16_t, 2048> bVec = {0};
        
        return std::make_unique<OpticalEncoderSim>(simHandler, aVec, bVec, A3, A4, 4096.0f * 10.0f / 1 * 11.0f / 62 * 14.0f / 48 * 13.0f / 45 * 1.0f / 42);
    }

    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()
    {
        constexpr static std::array<int16_t, 513> compVec = {0};
        return std::make_unique<ResistiveEncoderSim>(simHandler, A1, 4096.0f * 220.0f / 360, compVec);
    }

    static std::unique_ptr<CurrentController> createCurrentController()
    {
        constexpr float pwmToStallCurrent{1.0f};
        constexpr float backEmfCurrent{0.0f};

        auto pwmHighFrqCompFun = [](uint16_t in)
        {
            return in;
        };
        auto pwm = std::make_unique<PwmHandlerSim>(simHandler, pwmHighFrqCompFun);
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
            K << -1.9824914858583672e-23f, 6.808150830005707e-19f, -1.0246881046334478e-14f, 8.979468650052665e-11f, -5.028779989915877e-07f, 0.0017548254273752164f, 0.0014220749321045931f,
                2.511086124189635e-19f, -7.0053853588061025e-15f, 7.649342768550831e-11f, -3.997017418475635e-07f, 0.0008714473464777145f, 0.4337783328664848f, -61.68698076064478f,
                5.268302746974665e-17f, -1.9704278703366155e-12f, 2.9001435813756453e-08f, -0.00020808122063929412f, 0.6790744641735348f, -286.70053636075215f, 35440.208679119176f;

            return K;
        }

        //system model A matrix
        static Eigen::Matrix3f getAMatrix()
        {
            Eigen::Matrix3f A;
            A << 1.0f, 0.0006f, 1.7999999999999997e-07f,
                0.0f, 1.0f, 0.0006f,
                0.0f, 0.0f, 1.0f;

            return A;
        }

        //system model invers A matrix
        static Eigen::Matrix3f getAInvMatrix()
        {
            Eigen::Matrix3f AInv;
            AInv << 1.0f, -0.0006f, 1.7999999999999997e-07f,
                0.0f, 1.0f, -0.0006f,
                0.0f, 0.0f, 1.0f;

            return AInv;
        }

        //system model B matrix
        static Eigen::Vector3f getBVector()
        {
            Eigen::Vector3f B;
            B << 1.7999999999999997e-07f,
                0.0006f,
                0.0f;

            return B;
        }

        //system model friction comp value
        static float getFrictionComp()
        {
            return 0.0f;
        }
    };
};

SimulationHandler SetupConfigHolder::simHandler;

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

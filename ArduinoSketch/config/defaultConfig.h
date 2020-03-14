#include <Eigen.h>
#include "../EncoderHandler.h"
#include "../ArduinoC++BugFixes.h"
#include "../CommunicationHandlers.h"

#ifndef CONFIG_HOLDER_H
#define CONFIG_HOLDER_H

class ConfigHolder
{
public:
    static constexpr float getMainEncoderGearRation()
    {
        return 1.0;
    }

    static std::unique_ptr<EncoderHandlerInterface> createMainEncoderHandler()
    {
        return std::make_unique<EncoderHandler>(A5);
    }

    static std::unique_ptr<EncoderHandlerInterface> createOutputEncoderHandler()
    {
        return std::unique_ptr<EncoderHandlerInterface>(nullptr);
    }

    static constexpr unsigned char getCommunicationId()
    {
        return 1;
    }

    static std::unique_ptr<CommunicationInterface> getCommunicationHandler()
    {
        return std::make_unique<DCServoCommunicationHandler>(getCommunicationId(), 115200);
    }

    static Eigen::Matrix<float, 5, 1> getControlParameterVector()
    {
        Eigen::Matrix<float, 5, 1> L;
        L << 24.628722042909875, 3.422417759025543, -0.18915403084733035, -0.18915403084733035 * 10, 1;

        return L;
    }

    class KalmanFilter
    {
      public:
        static Eigen::Vector3f getKVector()
        {
            Eigen::Vector3f K;
            
            //K << 0.09799424580193117,
            //    2.4964409292865963,
            //    0.5652699951929046;

            //K << 0.16531265413057228,
            //    7.247273820257207,
            //    2.5269719748600346;

            //K << 0.1983825910518331,
            //    10.478680647096779,
            //    4.291072093854287;

            //K << 0.32684007265646386,
            //    28.567812385253276,
            //    18.53313095588553;

            //K << 0.3888374995706161,
            //    40.40850366768767,
            //    30.93862171115473;

            //K << 0.4791413260700308,
            //    61.21885215942826,
            //    57.39263621353391;

            //K << 0.5085413648625792,
            //    68.89605951297686,
            //    68.47216577817979;

            //K << 0.622743534980963,
            //    102.86198025189901,
            //    124.93406815357332;

            // 30 * 4 * 2 pole
            K << 0.7316969391112194,
                141.316769493361,
                201.79065187534968;

            // 60 * 4 * 2 pole
            //K << 1.2879572201280896,
            //    425.2887752180842,
            //    1089.0860221513021;

            // 100 * 4 * 2 pole
            //K << 1.8229097365190623,
            //    825.8706566376411,
            //    3075.4863531454275;

            return K;
        }


        static Eigen::Matrix3f getAMatrix()
        {
            Eigen::Matrix3f A;
            A << 1.0, 0.0012, 3.6719999999999994e-05,
            0.0, 0.994, 0.0612,
            0.0, 0.0, 1.0;

            return A;
        }

        static Eigen::Matrix3f getAInvMatrix()
        {
            Eigen::Matrix3f AInv;

            AInv << 1.0, -0.0012072434607645873, 3.7163299798792755e-05,
                0.0, 1.0060362173038229, -0.06156941649899396,
                0.0, 0.0, 1.0;

            return AInv;
        }

        static Eigen::Vector3f getBVector()
        {
            Eigen::Vector3f B;

            B << 3.6719999999999994e-05,
                0.0612,
                0.0;

            return B;
        }
    };
};

#endif

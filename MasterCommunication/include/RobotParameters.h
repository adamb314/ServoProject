#include <Eigen/Dense>
#include <cmath>

#include "DummyTrajectoryGenerator.h"

#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

using EigenVectord3 = Eigen::Matrix<double, 3, 1>;
using EigenMatrixd3 = Eigen::Matrix<double, 3, 3>;
using EigenVectord6 = Eigen::Matrix<double, 6, 1>;
using EigenMatrixd6 = Eigen::Matrix<double, 6, 6>;

namespace RobotParameters
{
    extern const EigenVectord3 zeroVec;
    extern const EigenVectord3 ex;
    extern const EigenVectord3 ey;
    extern const EigenVectord3 ez;

    extern const double scalarGravity;
    extern const EigenVectord3 gravity;

    extern const EigenVectord3 s1Translation;
    extern const EigenVectord3 s1RotationAxis;
    EigenMatrixd3 s1Rotation(double rad);

    extern const EigenVectord3 s2Translation;
    extern const EigenVectord3 s2RotationAxis;
    EigenMatrixd3 s2Rotation(double rad);

    extern const EigenVectord3 s3Translation;
    extern const EigenVectord3 s3RotationAxis;
    EigenMatrixd3 s3Rotation(double rad);

    extern const EigenVectord3 s4Translation;
    extern const EigenVectord3 s4RotationAxis;
    EigenMatrixd3 s4Rotation(double rad);

    extern const double s5TranslationLength;
    extern const EigenVectord3 s5Translation;
    extern const EigenVectord3 s5RotationAxis;
    EigenMatrixd3 s5Rotation(double rad);

    extern const EigenVectord3 s6ZeroRotationDir;
    extern const EigenVectord3 s6RotationAxis;
    EigenMatrixd3 s6Rotation(double rad);

    struct DynamicRetType
    {
        EigenVectord3 m;
        EigenVectord3 f;
        double tau;
    };

    struct DynamicMatrices
    {
        EigenMatrixd6 inert{};
        EigenMatrixd6 w2Torque{};
        EigenVectord6 externalTorque{};
    };

    auto s1Dynamic(const EigenVectord3& wacc, const EigenVectord3& wvel, const EigenVectord3& acc, const EigenVectord3& me, const EigenVectord3& fe);

    auto s2Dynamic(const EigenVectord3& wacc, const EigenVectord3& wvel, const EigenVectord3& acc, const EigenVectord3& me, const EigenVectord3& fe);

    auto s3Dynamic(const EigenVectord3& wacc, const EigenVectord3& wvel, const EigenVectord3& acc, const EigenVectord3& me, const EigenVectord3& fe);

    EigenVectord3 combinedDynamic(const EigenVectord3& s1Wacc, const EigenVectord3& s2Wacc, const EigenVectord3& s3Wacc,
            const EigenVectord3& s1Wvel, const EigenVectord3& s2Wvel, const EigenVectord3& s3Wvel,
            const EigenVectord3& s1Acc, const EigenVectord3& s2Acc, const EigenVectord3& s3Acc,
            const EigenMatrixd3& s2RotationMatrix,
            const EigenMatrixd3& s3RotationMatrix);

    DynamicMatrices getDynamicMatrices(const EigenVectord6& a);

    class DynamicRobotDynamics : public RobotDynamics<6, double>
    {
      public:
        static const double pwmLimit;
        static const double currentToTorqueScale;
        static const double gearBoxMomentOfInertia;
        static const double pwmToStallCurrent;
        static const double backEmfCurrent;

        DynamicRobotDynamics(double dtTime);

        virtual ~DynamicRobotDynamics();

        virtual void update(const Eigen::Matrix<double, 6, 1>& pos, const Eigen::Matrix<double, 6, 1>& preVelDir, const Eigen::Matrix<double, 6, 1>& postVelDir,
            double approxVel);

        void recalculateFreedForward(TrajectoryItem<6, double>& itemK, const TrajectoryItem<6, double>& itemKp1);

        Eigen::Matrix<double, 6, 1> recalcPwm(const Eigen::Matrix<double, 6, 1>& current, const Eigen::Matrix<double, 6, 1>& vel);
    };
};

#endif

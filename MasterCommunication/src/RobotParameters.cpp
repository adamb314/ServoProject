#include "RobotParameters.h"

namespace RobotParameters
{
    const EigenVectord3 zeroVec{0, 0, 0};
    const EigenVectord3 ex{1,0,0};
    const EigenVectord3 ey{0,1,0};
    const EigenVectord3 ez{0,0,1};

    const double scalarGravity = 9.81;
    const EigenVectord3 gravity{scalarGravity * ez};

    const EigenVectord3 s1Translation{0.015 * ex};
    const EigenVectord3 s1RotationAxis{ez};
    EigenMatrixd3 s1Rotation(double rad)
    {
        EigenMatrixd3 out;
        out << cos(rad), -sin(rad), 0,
               sin(rad), cos(rad), 0,
               0, 0, 1;
        return out;
    }

    const EigenVectord3 s2Translation{-0.2 * ey};
    const EigenVectord3 s2RotationAxis{-ex};
    EigenMatrixd3 s2Rotation(double rad)
    {
        EigenMatrixd3 out;
        out << 1, 0, 0,
               0, cos(rad), sin(rad),
               0, -sin(rad), cos(rad);
        return out;
    }

    const EigenVectord3 s3Translation{-(0.2 + 0.027 + 0.007) * ey};
    const EigenVectord3 s3RotationAxis{ex};
    EigenMatrixd3 s3Rotation(double rad)
    {
        EigenMatrixd3 out;
        out << 1, 0, 0,
               0, cos(rad), -sin(rad),
               0, sin(rad), cos(rad);
        return out;
    }

    const EigenVectord3 s4Translation{zeroVec};
    const EigenVectord3 s4RotationAxis{ey};
    EigenMatrixd3 s4Rotation(double rad)
    {
        EigenMatrixd3 out;
        out << cos(rad), 0, sin(rad),
               0, 1, 0,
               -sin(rad), 0, cos(rad);
        return out;
    }

    const double s5TranslationLength{0.029};
    const EigenVectord3 s5Translation{-s5TranslationLength * ey};
    const EigenVectord3 s5RotationAxis{-ex};
    EigenMatrixd3 s5Rotation(double rad)
    {
        EigenMatrixd3 out;
        out << 1, 0, 0,
               0, cos(rad), sin(rad),
               0, -sin(rad), cos(rad);
        return out;
    }

    const EigenVectord3 s6ZeroRotationDir{-ez};
    const EigenVectord3 s6RotationAxis{ey};
    EigenMatrixd3 s6Rotation(double rad)
    {
        EigenMatrixd3 out;
        out << cos(rad), 0, sin(rad),
               0, 1, 0,
               -sin(rad), 0, cos(rad);
        return out;
    }

    auto s1Dynamic(const EigenVectord3& wacc, const EigenVectord3& wvel, const EigenVectord3& acc, const EigenVectord3& me, const EigenVectord3& fe)
    {
        DynamicRetType out{};

        double mass = 0.1;
        EigenVectord3 mcDist{0, 0, 0};
        EigenMatrixd3 I;
        I << 0, 0, 0,
             0, 0, 0,
             0, 0, 0;
        EigenVectord3 mcAcc = acc + wacc.cross(mcDist) + wvel.cross(wvel.cross(mcDist));

        out.f = mass * mcAcc + fe;

        out.m = I * wacc + s1Translation.cross(fe) - mass * mcAcc.cross(mcDist) + me;

        out.tau = s1RotationAxis.dot(out.m);

        return out;
    }

    auto s2Dynamic(const EigenVectord3& wacc, const EigenVectord3& wvel, const EigenVectord3& acc, const EigenVectord3& me, const EigenVectord3& fe)
    {
        DynamicRetType out{};

        const double mass = 0.18;
        const double mcDistLength = 0.1;
        const double pendulumResFre = 10.0 / (8 + 3 * 1 / 30.0);
        const double pendulumResW = 2 * M_PI * pendulumResFre;
        const double inertia = (mass * scalarGravity * mcDistLength) / (pendulumResW  * pendulumResW) - mass * mcDistLength * mcDistLength;
        EigenVectord3 mcDist{0, -mcDistLength, 0};
        EigenMatrixd3 I;
        I << inertia, 0, 0,
             0, 0, 0,
             0, 0, inertia;
        I *= mass * mcDistLength * mcDistLength;
        EigenVectord3 mcAcc = acc + wacc.cross(mcDist) + wvel.cross(wvel.cross(mcDist));

        out.f = mass * mcAcc + fe;

        out.m = I * wacc + s2Translation.cross(fe) - mass * mcAcc.cross(mcDist) + me;

        out.tau = s2RotationAxis.dot(out.m);

        return out;
    }

    auto s3Dynamic(const EigenVectord3& wacc, const EigenVectord3& wvel, const EigenVectord3& acc, const EigenVectord3& me, const EigenVectord3& fe)
    {
        DynamicRetType out{};

        const double mass = 0.069;
        const double mcDistLength = 0.154;
        const double pendulumResFre = 30.0 / 27.54;
        const double pendulumResW = 2 * M_PI * pendulumResFre;
        const double inertia = (mass * scalarGravity * mcDistLength) / (pendulumResW  * pendulumResW) - mass * mcDistLength * mcDistLength;
        EigenVectord3 mcDist{0, -mcDistLength, 0};
        EigenMatrixd3 I;
        I << inertia, 0, 0,
             0, 0, 0,
             0, 0, inertia;
        I *= mass * mcDistLength * mcDistLength;
        EigenVectord3 mcAcc = acc + wacc.cross(mcDist) + wvel.cross(wvel.cross(mcDist));

        out.f = mass * mcAcc + fe;

        out.m = I * wacc + s3Translation.cross(fe) - mass * mcAcc.cross(mcDist) + me;

        out.tau = s3RotationAxis.dot(out.m);

        return out;
    }

    EigenVectord3 combinedDynamic(const EigenVectord3& s1Wacc, const EigenVectord3& s2Wacc, const EigenVectord3& s3Wacc,
            const EigenVectord3& s1Wvel, const EigenVectord3& s2Wvel, const EigenVectord3& s3Wvel,
            const EigenVectord3& s1Acc, const EigenVectord3& s2Acc, const EigenVectord3& s3Acc,
            const EigenMatrixd3& s2RotationMatrix,
            const EigenMatrixd3& s3RotationMatrix)
    {
        auto [ms3, fs3, taus3] = s3Dynamic(s3Wacc, s3Wvel, s3Acc,
                EigenVectord3{0, 0, 0},
                EigenVectord3{0, 0, 0});
        ms3 = s3RotationMatrix * ms3;
        fs3 = s3RotationMatrix * fs3;
        auto [ms2, fs2, taus2] = s2Dynamic(s2Wacc, s2Wvel, s2Acc, ms3, fs3);
        ms2 = s2RotationMatrix * ms2;
        fs2 = s2RotationMatrix * fs2;
        auto [ms1, fs1, taus1] = s1Dynamic(s1Wacc, s1Wvel, s1Acc, ms2, fs2);

        return EigenVectord3{taus1, taus2, taus3};
    }

    DynamicMatrices getDynamicMatrices(const EigenVectord6& a)
    {
        EigenMatrixd3 s2RotationM = s2Rotation(a[1]);
        EigenMatrixd3 s3RotationM = s3Rotation(a[2]);
        EigenMatrixd3 s1InvRotationM = s1Rotation(-a[0]);
        EigenMatrixd3 s2InvRotationM = s2Rotation(-a[1]);
        EigenMatrixd3 s3InvRotationM = s3Rotation(-a[2]);

        EigenVectord3 s1Acc = s1InvRotationM * gravity;
        EigenVectord3 s2Acc = s2InvRotationM * s1Acc;
        EigenVectord3 s3Acc = s3InvRotationM * s2Acc;

        EigenVectord3 accTau = combinedDynamic(zeroVec, zeroVec, zeroVec,
                zeroVec, zeroVec, zeroVec,
                s1Acc, s2Acc, s3Acc,
                s2RotationM, s3RotationM);

        EigenVectord3 s1s1Wacc = s1RotationAxis;

        EigenVectord3 s1s2Wacc = s2InvRotationM * s1s1Wacc;
        EigenVectord3 s1s2DistToC = s2InvRotationM * s1Translation;

        EigenVectord3 s1s3Wacc = s3InvRotationM * s1s2Wacc;
        EigenVectord3 s1s3DistToC = s3InvRotationM * (s1s2DistToC + s2Translation);

        EigenVectord3 s1Tau = combinedDynamic(s1s1Wacc, s1s2Wacc, s1s3Wacc,
                zeroVec, zeroVec, zeroVec,
                zeroVec, s1s2Wacc.cross(s1s2DistToC), s1s3Wacc.cross(s1s3DistToC),
                s2RotationM, s3RotationM);

        EigenVectord3 s2s2Wacc = s2RotationAxis;

        EigenVectord3 s2s3Wacc = s3InvRotationM * s2s2Wacc;
        EigenVectord3 s2s3DistToC = s3InvRotationM * (s2Translation);

        EigenVectord3 s2Tau = combinedDynamic(zeroVec, s2s2Wacc, s2s3Wacc,
                zeroVec, zeroVec, zeroVec,
                zeroVec, zeroVec, s2s3Wacc.cross(s2s3DistToC),
                s2RotationM, s3RotationM);

        EigenVectord3 s3s3Wacc = s3RotationAxis;

        EigenVectord3 s3Tau = combinedDynamic(zeroVec, zeroVec, s3s3Wacc,
                zeroVec, zeroVec, zeroVec,
                zeroVec, zeroVec, zeroVec,
                s3RotationM, s3RotationM);


        EigenVectord3 s1Vels2DistToC = s2InvRotationM * s1Translation;
        EigenVectord3 s1Vels3DistToC = s3InvRotationM * (s1Vels2DistToC + s2Translation);

        EigenVectord3 s1Vels2Acc = s1RotationAxis.cross(s1RotationAxis.cross(s1Vels2DistToC));
        EigenVectord3 s1Vels3Acc = s1RotationAxis.cross(s1RotationAxis.cross(s1Vels3DistToC));

        EigenVectord3 s1s1Wvel = s1RotationAxis;
        EigenVectord3 s1s2Wvel = s2InvRotationM * s1s1Wvel;
        EigenVectord3 s1s3Wvel = s3InvRotationM * s1s2Wvel;

        EigenVectord3 s1WvelTau = combinedDynamic(zeroVec, zeroVec, zeroVec,
                s1s1Wvel, s1s2Wvel, s1s3Wvel,
                zeroVec, s1Vels2Acc, s1Vels3Acc,
                s2RotationM, s3RotationM);

        EigenVectord3 s2Vels3DistToC = s3InvRotationM * (s2Translation);
        EigenVectord3 s2Vels3Acc = s2RotationAxis.cross(s2RotationAxis.cross(s2Vels3DistToC));

        EigenVectord3 s2s2Wvel = s2RotationAxis;
        EigenVectord3 s2s3Wvel = s3InvRotationM * s2s2Wvel;

        EigenVectord3 s2WvelTau = combinedDynamic(zeroVec, zeroVec, zeroVec,
                zeroVec, s2s2Wvel, s2s3Wvel,
                zeroVec, zeroVec, s2Vels3Acc,
                s2RotationM, s3RotationM);

        EigenVectord3 s3s3Wvel = s3RotationAxis;

        EigenVectord3 s3WvelTau = combinedDynamic(zeroVec, zeroVec, zeroVec,
                zeroVec, zeroVec, s3s3Wvel,
                zeroVec, zeroVec, zeroVec,
                s2RotationM, s3RotationM);

        DynamicMatrices out;

        out.inert << s1Tau[0], s2Tau[0], s3Tau[0], 0, 0, 0,
                s1Tau[1], s2Tau[1], s3Tau[1], 0, 0, 0,
                s1Tau[2], s2Tau[2], s3Tau[2], 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;

        out.w2Torque << s1WvelTau[0], s2WvelTau[0], s3WvelTau[0], 0, 0, 0,
                s1WvelTau[1], s2WvelTau[1], s3WvelTau[1], 0, 0, 0,
                s1WvelTau[2], s2WvelTau[2], s3WvelTau[2], 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;

        out.externalTorque << accTau[0],
                accTau[1],
                accTau[2],
                0,
                0,
                0;

        return out;
    }

    const double DynamicRobotDynamics::pwmLimit = 1023;
    const double DynamicRobotDynamics::currentToTorqueScale = 0.00056;
    const double DynamicRobotDynamics::gearBoxMomentOfInertia = 0.015064771031078847;
    const double DynamicRobotDynamics::pwmToStallCurrent = 1.945991041784367;
    const double DynamicRobotDynamics::backEmfCurrent = -0.00030467666381376284 * 4096 / 2.0 / M_PI;

    DynamicRobotDynamics::DynamicRobotDynamics(double dtTime)
    {
        RobotDynamics<6, double>::dt = dtTime;
        RobotDynamics<6, double>::maxAxisAbsVel = {2, 2, 2, 2, 2, 2};
    };

    DynamicRobotDynamics::~DynamicRobotDynamics(){};

    void DynamicRobotDynamics::update(const Eigen::Matrix<double, 6, 1>& pos, const Eigen::Matrix<double, 6, 1>& preVelDir, const Eigen::Matrix<double, 6, 1>& postVelDir,
        double approxVel)
    {
        RobotDynamics<6, double>::torqueLimits = {pwmLimit, pwmLimit, pwmLimit, pwmLimit, pwmLimit, pwmLimit};

        auto dynamicMatrices = getDynamicMatrices({0,0,0,0,0,0});

        dynamicMatrices.inert += Eigen::Matrix<double, 6, 6>::Identity() * gearBoxMomentOfInertia;

        // wacc = inertInv * currentToTorqueScale * backEmfCurrent * pwmLimit * vk + inertInv * currentToTorqueScale *  pwmToStallCurrent * u - inertInv * w2torq * vk * vk - inertInv * externTorq
        auto inertInv = dynamicMatrices.inert.inverse();
        auto waccVk = inertInv * currentToTorqueScale * backEmfCurrent * pwmLimit;
        auto waccU = inertInv * currentToTorqueScale * pwmToStallCurrent;
        auto approxW2 = preVelDir;
        approxW2.normalize();
        approxW2 *= 0;//approxVel;
        //std::transform(std::cbegin(approxW2), std::cend(approxW2), std::begin(approxW2), [](auto& c){return c * c;});
        auto waccExtern = - inertInv * (dynamicMatrices.w2Torque * approxW2 + dynamicMatrices.externalTorque);

        //vkp1 = vk + dt * wacc
        RobotDynamics<6, double>::a = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;
        RobotDynamics<6, double>::a += RobotDynamics<6, double>::dt * waccVk;
        RobotDynamics<6, double>::b = RobotDynamics<6, double>::dt * waccU;
        RobotDynamics<6, double>::externalTorqueAcc = RobotDynamics<6, double>::dt * waccExtern;

        RobotDynamics<6, double>::bInv = RobotDynamics<6, double>::b.inverse();
    };

    void DynamicRobotDynamics::recalculateFreedForward(TrajectoryItem<6, double>& itemK, const TrajectoryItem<6, double>& itemKp1)
    {
        auto dynamicMatrices = getDynamicMatrices(itemK.p);

        dynamicMatrices.inert += Eigen::Matrix<double, 6, 6>::Identity() * gearBoxMomentOfInertia;

        // wacc = inertInv * currentToTorqueScale * backEmfCurrent * pwmLimit * vk + inertInv * currentToTorqueScale *  pwmToStallCurrent * u - inertInv * w2torq * vk * vk - inertInv * externTorq
        auto inertInv = dynamicMatrices.inert.inverse();
        auto waccU = inertInv * currentToTorqueScale;
        auto approxW2 = itemK.v;
        std::transform(std::cbegin(approxW2), std::cend(approxW2), std::begin(approxW2), [](auto& c){return c * c;});
        auto waccExtern = - inertInv * (dynamicMatrices.w2Torque * approxW2 + dynamicMatrices.externalTorque);

        //vkp1 = vk + dt * wacc
        auto a = Eigen::Matrix<double, 6, 6>::Identity() * 1.0;
        auto b = RobotDynamics<6, double>::dt * waccU;
        auto externalTorqueAcc = RobotDynamics<6, double>::dt * waccExtern;

        itemK.u = b.inverse() * (itemKp1.v - a * itemK.v - externalTorqueAcc);
    };

    Eigen::Matrix<double, 6, 1> DynamicRobotDynamics::recalcPwm(const Eigen::Matrix<double, 6, 1>& current, const Eigen::Matrix<double, 6, 1>& vel)
    {
        Eigen::Matrix<double, 6, 1> pwm;
        //current[i] = pwmToStallCurrent * pwm[i] + backEmfCurrent * vel[i] * pwm[i];
        for (size_t i = 0; i != 6; ++i)
        {
            pwm[i] = current[i] / (pwmToStallCurrent + backEmfCurrent * vel[i]);
        }
        return pwm;
    }
};

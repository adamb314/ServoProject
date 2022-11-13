#include <ArduinoEigenDense.h>
#include "ArduinoC++BugFixes.h"

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
protected:
    const Eigen::Matrix3f A;
    const Eigen::Vector3f B;
    const Eigen::Matrix3f AInv;
    const Eigen::Matrix<float, 3, 7> polyK;
    Eigen::Vector3f AInvXK;
    Eigen::Vector3f xhat{Eigen::Vector3f::Zero()};

public:
    KalmanFilter(const Eigen::Matrix3f& A,
            const Eigen::Vector3f& B,
            const Eigen::Matrix3f& AInv,
            const Eigen::Matrix<float, 3, 7>& polyK);

    template<typename T>
    static std::unique_ptr<KalmanFilter> create(bool approximation = false);

    void setFilterSpeed(float speed);
    Eigen::Vector3f calculateNewKVector(float filterSpeed) const;
    void setNewKVector(const Eigen::Vector3f& K);

    void reset(const Eigen::Vector3f& xhat0);

    virtual auto update(float y) -> decltype(xhat);
    virtual void postUpdate(float u);

protected:
    static Eigen::Matrix<float, 3, 7> translateKVecToPolyK(const Eigen::Matrix<float, 3, 7>& in)
    {
        return in;
    }

    static Eigen::Matrix<float, 3, 7> translateKVecToPolyK(const Eigen::Matrix<float, 3, 4>& in)
    {
        Eigen::Matrix<float, 3, 7> polyK;
        polyK << 0.0f, 0.0f, 0.0f, in(0, 0), in(0, 1), in(0, 2), in(0, 3),
            0.0f, 0.0f, 0.0f, in(1, 0), in(1, 1), in(1, 2), in(1, 3),
            0.0f, 0.0f, 0.0f, in(2, 0), in(2, 1), in(2, 2), in(2, 3);
        return polyK;
    }

    static Eigen::Matrix<float, 3, 7> translateKVecToPolyK(const Eigen::Vector3f& in)
    {
        Eigen::Matrix<float, 3, 7> polyK;
        polyK << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, in[0],
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, in[1],
            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, in[2];
        return polyK;
    }
};

class KalmanFilterApproximation : public KalmanFilter
{
public:
    KalmanFilterApproximation(const Eigen::Matrix3f& A,
            const Eigen::Vector3f& B,
            const Eigen::Matrix3f& AInv,
            const Eigen::Matrix<float, 3, 7>& polyK) :
        KalmanFilter(A, B, AInv, polyK)
    {
    }

    virtual auto update(float y) -> decltype(KalmanFilter::update(y)) override;
    virtual void postUpdate(float u) override;
};

template<typename T>
std::unique_ptr<KalmanFilter> KalmanFilter::create(bool approximation)
{
    auto A = T::getAMatrix();
    auto B = T::getBVector();
    auto AInv = T::getAInvMatrix();
    auto polyK = translateKVecToPolyK(T::getKVector());
    if (approximation)
    {
        return std::make_unique<KalmanFilterApproximation>(A, B, AInv, polyK);
    }

    return std::make_unique<KalmanFilter>(A, B, AInv, polyK);
}

#endif

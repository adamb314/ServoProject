#include <Eigen30.h>
#include "ArduinoC++BugFixes.h"

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
  private:
    Eigen::Matrix3f A;
    Eigen::Vector3f B;
    Eigen::Vector3f AInvXK;
    Eigen::Vector3f xhat{Eigen::Vector3f::Zero()};
    float frictionComp;

  public:
    KalmanFilter(const Eigen::Matrix3f& A,
            const Eigen::Vector3f& B,
            const Eigen::Vector3f& AInvXK,
            float frictionComp);

    template<typename T>
    static std::unique_ptr<KalmanFilter> create();

    void reset(const Eigen::Vector3f& xhat0);

    auto update(float u, float y) -> decltype(xhat);

    const auto getA() const -> const decltype(A) &
    {
        return A;
    }

    const auto getB() const -> const decltype(B) &
    {
        return B;
    }

    const auto getX() const -> const decltype(xhat) &
    {
        return xhat;
    }

    float getFrictionComp() const
    {
        return frictionComp;
    }

    uint32_t getCycleTimeUs()
    {
        return static_cast<uint32_t>(A(0, 1) * 1000000ul);
    }
};

template<typename T>
std::unique_ptr<KalmanFilter> KalmanFilter::create()
{
    return std::make_unique<KalmanFilter>(T::getAMatrix(),
        T::getBVector(),
        T::getAInvMatrix() *
            T::getKVector(),
        T::getFrictionComp());
}

#endif

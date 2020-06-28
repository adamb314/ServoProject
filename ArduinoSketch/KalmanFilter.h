#include <Eigen30.h>

#include "config/config.h"

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
  private:
    Eigen::Matrix3f A;
    Eigen::Vector3f B;
    Eigen::Vector3f K;
    Eigen::Vector3f xhat;

  public:
    KalmanFilter();

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

    const auto getK() const -> const decltype(K) &
    {
        return K;
    }

    const auto getX() const -> const decltype(xhat) &
    {
        return xhat;
    }
};

#endif

#include <Eigen30.h>
#include "ArduinoC++BugFixes.h"

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
  private:
    const Eigen::Matrix3f A;
    const Eigen::Vector3f B;
    const Eigen::Matrix3f AInv;
    const Eigen::Matrix<float, 3, 4> polyK;
    Eigen::Vector3f AInvXK;
    Eigen::Vector3f xhat{Eigen::Vector3f::Zero()};

  public:
    KalmanFilter(const Eigen::Matrix3f& A,
            const Eigen::Vector3f& B,
            const Eigen::Matrix3f& AInv,
            const Eigen::Matrix<float, 3, 4>& polyK);

    template<typename T>
    static std::unique_ptr<KalmanFilter> create();

    void setFilterSpeed(float speed);

    void reset(const Eigen::Vector3f& xhat0);

    auto update(float u, float y) -> decltype(xhat);

private:
    static Eigen::Matrix<float, 3, 4> translateKVecToPolyK(const Eigen::Matrix<float, 3, 4>& in)
    {
        return in;
    }

    static Eigen::Matrix<float, 3, 4> translateKVecToPolyK(const Eigen::Vector3f& in)
    {
        Eigen::Matrix<float, 3, 4> polyK;
        polyK << 0.0, 0.0, 0.0, in[0],
            0.0, 0.0, 0.0, in[1],
            0.0, 0.0, 0.0, in[2];
        return polyK;
    }
};


template<typename T>
std::unique_ptr<KalmanFilter> KalmanFilter::create()
{
    return std::make_unique<KalmanFilter>(T::getAMatrix(),
        T::getBVector(),
        T::getAInvMatrix(),
        translateKVecToPolyK(T::getKVector()));
}

#endif

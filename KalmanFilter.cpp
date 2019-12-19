#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    Eigen::Vector3f xhat0;
    xhat0 << 0, 0, 0;
    reset(xhat0);

    Eigen::Matrix3f AInv;

    A = ConfigHolder::KalmanFilter::getAMatrix();

    AInv = ConfigHolder::KalmanFilter::getAInvMatrix();

    B = ConfigHolder::KalmanFilter::getBVector();

    K = ConfigHolder::KalmanFilter::getKVector();

    K = AInv * K;
}

void KalmanFilter::reset(const Eigen::Vector3f& xhat0)
{
    xhat = xhat0;
}

auto KalmanFilter::update(float u, float y) -> decltype(xhat)
{
    xhat += K * (y - xhat[0]);
    Eigen::Vector3f out = xhat;
    xhat = A * xhat + B * u;
    return out;
}

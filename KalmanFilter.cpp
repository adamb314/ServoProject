#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    Eigen::Vector3f xhat0;
    xhat0 << 0, 0, 0;
    reset(xhat0);

    A << 1.0, 0.0012, 3.6719999999999994e-05,
        0.0, 0.995, 0.0612,
        0.0, 0.0, 1.0;

    Eigen::Matrix3f AInv;
    AInv << 1.0, -0.0012060301507537687, 3.708904522613064e-05,
        0.0, 1.0050251256281406, -0.0615075376884422,
        0.0, 0.0, 1.0;

    B << 3.6719999999999994e-05,
        0.0612,
        0.0;

    K << 0.3898374995701852,
        40.72789607002568,
        30.954052444874385;

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

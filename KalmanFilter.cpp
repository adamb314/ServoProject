#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    Eigen::Vector3f xhat0;
    xhat0 << 0, 0, 0;
    reset(xhat0);

    Eigen::Matrix3f AInv;

    A << 1.0, 0.0012, 3.6719999999999994e-05,
        0.0, 0.994, 0.0612,
        0.0, 0.0, 1.0;

    AInv << 1.0, -0.0012072434607645873, 3.7163299798792755e-05,
        0.0, 1.0060362173038229, -0.06156941649899396,
        0.0, 0.0, 1.0;

    B << 3.6719999999999994e-05,
        0.0612,
        0.0;

    K << 0.09799424580193117,
        2.4964409292865963,
        0.5652699951929046;

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

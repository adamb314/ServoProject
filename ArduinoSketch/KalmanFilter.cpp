#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(const Eigen::Matrix3f& A,
            const Eigen::Vector3f& B,
            const Eigen::Matrix3f& AInv,
            const Eigen::Matrix<float, 3, 4>& polyK) :
    A(A), B(B), AInv(AInv), polyK(polyK)
{
	setFilterSpeed(20 * 4 * 4);
}

void KalmanFilter::setFilterSpeed(float speed)
{
	float speed3 = speed * speed * speed;
	float speed2 = speed * speed;

    Eigen::Vector3f K;
    K << polyK(0, 0) * speed3 + polyK(0, 1) * speed2 + polyK(0, 2) * speed + polyK(0, 3),
    	polyK(1, 0) * speed3 + polyK(1, 1) * speed2 + polyK(1, 2) * speed + polyK(1, 3),
    	polyK(2, 0) * speed3 + polyK(2, 1) * speed2 + polyK(2, 2) * speed + polyK(2, 3);

    AInvXK = AInv * K;
}

void KalmanFilter::reset(const Eigen::Vector3f& xhat0)
{
    xhat = xhat0;
}

auto KalmanFilter::update(float u, float y) -> decltype(xhat)
{
    xhat += AInvXK * (y - xhat[0]);
    Eigen::Vector3f out = xhat;
    xhat = A * xhat + B * u;
    return out;
}

auto KalmanFilterApproximation::update(float u, float y) -> decltype(KalmanFilter::update(u, y))
{
    xhat[1] += AInvXK[1] * (y - xhat[0]);
    xhat[0] = y;
    Eigen::Vector3f out = xhat;
    xhat[0] += A(0, 1) * xhat[1];
    return out;
}

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(const Eigen::Matrix3f& A,
            const Eigen::Vector3f& B,
            const Eigen::Matrix3f& AInv,
            const Eigen::Matrix<float, 3, 7>& polyK) :
    A(A), B(B), AInv(AInv), polyK(polyK)
{
	setFilterSpeed(20 * 4 * 4);
}

void KalmanFilter::setFilterSpeed(float speed)
{
	float speed2 = speed * speed;
    float speed3 = speed2 * speed;
    float speed4 = speed3 * speed;
    float speed5 = speed4 * speed;
	float speed6 = speed5 * speed;

    Eigen::Vector3f K;
    K << polyK(0, 0) * speed6 + polyK(0, 1) * speed5 + polyK(0, 2) * speed4 + polyK(0, 3) * speed3 + polyK(0, 4) * speed2 + polyK(0, 5) * speed + polyK(0, 6),
    	polyK(1, 0) * speed6 + polyK(1, 1) * speed5 + polyK(1, 2) * speed4 + polyK(1, 3) * speed3 + polyK(1, 4) * speed2 + polyK(1, 5) * speed + polyK(1, 6),
    	polyK(2, 0) * speed6 + polyK(2, 1) * speed5 + polyK(2, 2) * speed4 + polyK(2, 3) * speed3 + polyK(2, 4) * speed2 + polyK(2, 5) * speed + polyK(2, 6);

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

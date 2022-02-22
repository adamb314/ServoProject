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
    auto K = calculateNewKVector(speed);
    setNewKVector(K);
}

Eigen::Vector3f KalmanFilter::calculateNewKVector(float filterSpeed) const
{
    const float& s = filterSpeed;
	const float s2 = s * s;
    const float s3 = s2 * s;
    const float s4 = s3 * s;
    const float s5 = s4 * s;
	const float s6 = s5 * s;

    Eigen::Vector3f K;
    K << polyK(0, 0) * s6 + polyK(0, 1) * s5 + polyK(0, 2) * s4 + polyK(0, 3) * s3 + polyK(0, 4) * s2 + polyK(0, 5) * s + polyK(0, 6),
    	polyK(1, 0) * s6 + polyK(1, 1) * s5 + polyK(1, 2) * s4 + polyK(1, 3) * s3 + polyK(1, 4) * s2 + polyK(1, 5) * s + polyK(1, 6),
    	polyK(2, 0) * s6 + polyK(2, 1) * s5 + polyK(2, 2) * s4 + polyK(2, 3) * s3 + polyK(2, 4) * s2 + polyK(2, 5) * s + polyK(2, 6);

    return K;
}

void KalmanFilter::setNewKVector(const Eigen::Vector3f& K)
{
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

#include <kalman/kalman.h>
#include <assert.h>

KalmanFilter::KalmanFilter()
{
}

void KalmanFilter::setMeasurementModel(const Eigen::MatrixXd& _C, const Eigen::MatrixXd& _Q)
{
    C = _C;
    Q = _Q;
}

void KalmanFilter::setTransitionModel(const Eigen::MatrixXd& _A, const Eigen::MatrixXd& _B, const Eigen::MatrixXd& _R)
{
    A = _A;
    B = _B;
    R = _R;
}

void KalmanFilter::setMeasurementCovariance(const Eigen::MatrixXd& _Q)
{
	Q = _Q;
}

void KalmanFilter::setTransitionCovariance(const Eigen::MatrixXd& _R)
{
	R = _R;
}

void KalmanFilter::getMeasurementModel(Eigen::MatrixXd& _C, Eigen::MatrixXd& _Q)
{
    _C = C;
    _Q = Q;
}

void KalmanFilter::getTransitionModel(Eigen::MatrixXd& _A, Eigen::MatrixXd& _B, Eigen::MatrixXd& _R)
{
    _A = A;
    _B = B;
    _R = R;
}

void KalmanFilter::update(Eigen::VectorXd& u, Eigen::MatrixXd& E,
                          const Eigen::VectorXd& u_t, const Eigen::VectorXd& z_t)
{
    Eigen::VectorXd u_p = u;
    Eigen::MatrixXd E_p = E;
    Eigen::VectorXd u_ = A*u_p + B*u_t;
    Eigen::MatrixXd E_ = A*E_p*A.transpose() + R;
    Eigen::MatrixXd K_t = E_*C.transpose()*(C*E_*C.transpose() + Q).inverse();
    u = u_ + K_t*(z_t - C*u_);
    E = (Eigen::MatrixXd::Identity(E.rows(), E.cols()) - K_t*C)*E;
}

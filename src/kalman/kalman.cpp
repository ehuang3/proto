#include <kalman/kalman.h>
#include <iostream>
#include <assert.h>

log4cxx::LoggerPtr KalmanFilter::logger(log4cxx::Logger::getLogger("KalmanFilter"));

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

void KalmanFilter::update(Eigen::VectorXd& x, Eigen::MatrixXd& E,
                          const Eigen::VectorXd& u_t, const Eigen::VectorXd& z_t)
{
	LOG4CXX_TRACE(logger, "============ Entering update ============");
    Eigen::VectorXd x_p = x;
    LOG4CXX_TRACE(logger, "x_t-1 = \n" << x_p);

    Eigen::MatrixXd E_p = E;
    LOG4CXX_TRACE(logger, "E_t-1 = \n" << E_p);

    LOG4CXX_TRACE(logger, "u_t = \n" << u_t);
    LOG4CXX_TRACE(logger, "z_t = \n" << z_t);

    Eigen::VectorXd x_ = A*x_p + B*u_t;
    LOG4CXX_TRACE(logger, "x_ = A*x_t-1 + B*u_t = \n" << x_);

    Eigen::MatrixXd E_ = A*E_p*A.transpose() + R;
    LOG4CXX_TRACE(logger, "E_ = A_t*E_t-1*A_t^T + R_t = \n" << E_);

    Eigen::MatrixXd K_t = E_*C.transpose()*(C*E_*C.transpose() + Q).inverse();
    LOG4CXX_TRACE(logger, "K_t = E_*C^T*(C*E_*C^T + Q)^-1 = \n" << K_t);

    x = x_ + K_t*(z_t - C*x_);
    LOG4CXX_TRACE(logger, "x_t = x_ + K_t(z_t - C*x_) = \n" << x);

    E = (Eigen::MatrixXd::Identity(E.rows(), E.cols()) - K_t*C)*E_;
    LOG4CXX_TRACE(logger, "E_t = (I - K_t*C)E_ = \n" << E);
}

std::ostream& operator<<(std::ostream& ostr, const KalmanFilter& filter)
{
	// x_t^bar = A_t*x_t-1 + B_t*u_t
	// E_t^bar = A_t*E_t-1*A_t^T + R_t
	// K_t = E_t^bar*C_t^T*(C_t*E_t^bar*C_t^T + Q_t)^-1
	// x_t = x_t^bar + K_t(z_t - C_t*x_t^bar)
	// E_t = (I - K_t*C_t)E_t^bar
	return ostr
		<< "A = \n" << filter.A << "\n"
		<< "B = \n" << filter.B << "\n"
		<< "R = \n" << filter.R << "\n"
		<< "C = \n" << filter.C << "\n"
		<< "Q = \n" << filter.Q;
}

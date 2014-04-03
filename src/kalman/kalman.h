#pragma once
#include <Eigen/Dense>
#include <log4cxx/logger.h>

class KalmanFilter
{
public:
    KalmanFilter();

    void setMeasurementModel(const Eigen::MatrixXd& _C, const Eigen::MatrixXd& _Q);
    void setTransitionModel(const Eigen::MatrixXd& _A, const Eigen::MatrixXd& _B, const Eigen::MatrixXd& _R);

	void setMeasurementCovariance(const Eigen::MatrixXd& _Q);
	void setTransitionCovariance(const Eigen::MatrixXd& _R);

    void getMeasurementModel(Eigen::MatrixXd& _C, Eigen::MatrixXd& _Q);
    void getTransitionModel(Eigen::MatrixXd& _A, Eigen::MatrixXd& _B, Eigen::MatrixXd& _R);

    void update(Eigen::VectorXd& u, Eigen::MatrixXd& E,
                const Eigen::VectorXd& u_t, const Eigen::VectorXd& z_t);

	friend std::ostream& operator<<(std::ostream& ostr, const KalmanFilter& filter);

	static log4cxx::LoggerPtr logger;

protected:
    Eigen::MatrixXd A, B, R, C, Q;
};

// Local Variables:
// mode: c++
// End:

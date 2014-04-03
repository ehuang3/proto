#include <iostream>
#include <cmath>
#include <gtest/gtest.h>
#include <gnuplot/gnuplot_i.hpp>
#include <utils/ProtoPaths.h>
#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "\t[", "]")
#include <kalman/kalman.h>
#include "log4cxx/basicconfigurator.h"
#include <log4cxx/xml/domconfigurator.h>
#include "log4cxx/helpers/exception.h"


using namespace std;
// namespace Eigen { typedef Matrix<double,1,1> Vector1d; }

/* ********************************************************************************************* */
void wait_for_key () // Programm halts until keypress
{
    cout << endl << "Press ENTER to continue..." << endl;
    std::cin.clear();
    std::cin.ignore(std::cin.rdbuf()->in_avail());
    std::cin.get();
    return;
}
/* ********************************************************************************************* */
KalmanFilter filter_1D;
static KalmanFilter* GENERATE_FILTER_1D() {

	Eigen::MatrixXd A(1,1), B(1,1), R(1,1);
	Eigen::MatrixXd C(1,1), Q(1,1);

	A(0,0) = 1;                 // State model
	B(0,0) = 1;                 // Command model
	R(0,0) = 0.5;               // Covariance of posterior state: A_t*x_t-1 + B_t*u_t

	C(0,0) = 1;                 // Measurement model
	Q(0,0) = 0.2;               // Measurement noise model

	filter_1D.setTransitionModel(A, B, R);
	filter_1D.setMeasurementModel(C, Q);

	return &filter_1D;
}
/* ********************************************************************************************* */
KalmanFilter filter_1D_vel;
static KalmanFilter* GENERATE_FILTER_1D_VEL(double dt) {
	// x_t^bar = A_t*x_t-1 + B_t*u_t
	// E_t^bar = A_t*E_t-1*A_t^T + R_t
	// K_t = E_t^bar*C_t^T*(C_t*E_t^bar*C_t^T + Q_t)^-1
	// x_t = x_t^bar + K_t(z_t - C_t*x_t^bar)
	// E_t = (I - K_t*C_t)E_t^bar

	Eigen::MatrixXd A(2,2);     // x_t = A_t*x_t-1 + B_t*u_t + ep_t
	Eigen::MatrixXd B(2,2);     // 
	Eigen::MatrixXd	R(2,2);     // Cov[A_t*x_t-1 + B_t*u_t]
	Eigen::MatrixXd C(1,2);     // z_t = C_t*x_t + del_t
	Eigen::MatrixXd Q(1,1);     // Cov[del_t]

	A << 
		1, dt,
		0, 1;
	B.setZero();
	R << 
		0.5, 0,
		0, 0.5;
	C << 1, 0;
	Q << 0.5;

	filter_1D_vel.setTransitionModel(A, B, R);
	filter_1D_vel.setMeasurementModel(C, Q);

	return &filter_1D_vel;
}
/* ********************************************************************************************* */
Eigen::VectorXd GENERATE_SINUSOIDAL(double t, double dt, double f, double a, double t0=0.0) {
	int n = t/dt;
	Eigen::VectorXd data(n);
	for(int i=0; i < n; i++) {
		data(i) = a*std::sin(2*M_PI/f*(dt*i + t0));
	}
	return data;
}
/* ********************************************************************************************* */
Eigen::VectorXd GENERATE_LINEAR(double t, double dt, double v, double t0=0.0) {
	int n = t/dt;
	Eigen::VectorXd data(n);
	for(int i=0; i < n; i++) {
		data(i) = (dt*i + t0)*v;
	}
	return data;
}
/* ********************************************************************************************* */
TEST(KALMAN, TEST_GNUPLOT) {
	return;

	try {
		Gnuplot g1("lines");

		//
		// Slopes
		//
		cout << "*** plotting slopes" << endl;
		g1.set_title("Slopes\\nNew Line");

		cout << "y = x" << endl;
		g1.plot_slope(1.0,0.0,"y=x");

		cout << "y = 2*x" << endl;
		g1.plot_slope(2.0,0.0,"y=2x");

		cout << "y = -x" << endl;
		g1.plot_slope(-1.0,0.0,"y=-x");
		g1.unset_title();

		//
		// Equations
		//
		g1.reset_plot();
		cout << endl << endl << "*** various equations" << endl;

		cout << "y = sin(x)" << endl;
		g1.plot_equation("sin(x)","sine");

		cout << "y = log(x)" << endl;
		g1.plot_equation("log(x)","logarithm");

		cout << "y = sin(x) * cos(2*x)" << endl;
		g1.plot_equation("sin(x)*cos(2*x)","sine product");

		//
		// Styles
		//
		g1.reset_plot();
		cout << endl << endl << "*** showing styles" << endl;

		cout << "sine in points" << endl;
		g1.set_pointsize(0.8).set_style("points");
		g1.plot_equation("sin(x)","points");

		cout << "sine in impulses" << endl;
		g1.set_style("impulses");
		g1.plot_equation("sin(x)","impulses");

		cout << "sine in steps" << endl;
		g1.set_style("steps");
		g1.plot_equation("sin(x)","steps");

		//
		// Save to ps
		//
		g1.reset_all();
		cout << endl << endl << "*** save to ps " << endl;

		cout << "y = sin(x) saved to test_output.ps in working directory" << endl;
		g1.savetops("test_output");
		g1.set_style("lines").set_samples(300).set_xrange(0,5);
		g1.plot_equation("sin(12*x)*exp(-x)").plot_equation("exp(-x)");

		g1.showonscreen(); // window output	

        std::vector<double> x, y, y2, dy, z;

#define NPOINTS    50 // length of array

        for (int i = 0; i < NPOINTS; i++)  // fill double arrays x, y, z
        {
            x.push_back((double)i);             // x[i] = i
            y.push_back((double)i * (double)i); // y[i] = i^2
            z.push_back( x[i]*y[i] );           // z[i] = x[i]*y[i] = i^3
            dy.push_back((double)i * (double)i / (double) 10); // dy[i] = i^2 / 10
        }
        y2.push_back(0.00); y2.push_back(0.78); y2.push_back(0.97); y2.push_back(0.43);
        y2.push_back(-0.44); y2.push_back(-0.98); y2.push_back(-0.77); y2.push_back(0.02);

        Gnuplot g2;
        cout << "window 2: user defined points" << endl;
        g2.plot_x(y2,"points");
        g2.set_smooth().plot_x(y2,"cspline");
        g2.set_smooth("bezier").plot_x(y2,"bezier");
        g2.unset_smooth();
	} catch (GnuplotException ge) {
        cout << ge.what() << endl;
    }
}
/* ********************************************************************************************* */
TEST(KALMAN, TEST_1D) {
	KalmanFilter* filter = GENERATE_FILTER_1D();

	Eigen::VectorXd x_p(1);         // Mean and covariance at time t-1
	Eigen::MatrixXd E_p(1,1);       // .
	Eigen::VectorXd u_t(1), z_t(1); // Command and measurement at time t

	x_p(0) = 7.5;
	E_p(0) = 1.0;
	u_t(0) = 0.0;
	z_t(0) = 6.0;

	// std::cout << "x_p: \n" << x_p << std::endl;
	// std::cout << "E_p: \n" << E_p << std::endl;

	// std::cout << "filter->update()" << std::endl;
	filter->update(x_p, E_p, u_t, z_t);

	// std::cout << "x_p: \n" << x_p << std::endl;
	// std::cout << "E_p: \n" << E_p << std::endl;

	u_t(0) = 10.0;
	z_t(0) = 0.0;

	// std::cout << "filter->update()" << std::endl;
	filter->update(x_p, E_p, u_t, z_t);

	// std::cout << "x_p: \n" << x_p << std::endl;
	// std::cout << "E_p: \n" << E_p << std::endl;

	u_t(0) = 10.0;
	z_t(0) = 0.0;

	// std::cout << "filter->update()" << std::endl;
	filter->update(x_p, E_p, u_t, z_t);

	// std::cout << "x_p: \n" << x_p << std::endl;
	// std::cout << "E_p: \n" << E_p << std::endl;
}
/* ********************************************************************************************* */
TEST(KALMAN, TEST_1D_LINEAR_VEL) {

	// Initialize
	double t = 1.0;
	double dt = 0.1;
	double v = 1;
	int n = t/dt;

	KalmanFilter* filter = GENERATE_FILTER_1D_VEL(dt);

	std::cout << "1D FILTER = \n" << *filter << std::endl;

	Eigen::MatrixXd	R(2,2);     // Cov[A_t*x_t-1 + B_t*u_t]
	Eigen::MatrixXd Q(1,1);     // Cov[del_t]

	R << 
		0.1, 0.,
		0., 0.1;
	Q << 0.00001;

	filter->setTransitionCovariance(R);
	filter->setMeasurementCovariance(Q);

	std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > x;
	std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > E;
	x.push_back(Eigen::VectorXd::Zero(2,1));
	E.push_back(Eigen::MatrixXd::Identity(2,2));

	for(int i=0; i < x.size(); i++) {
		std::cout << "x(" << i << "):\n" << x[i] << std::endl;
		std::cout << "E(" << i << "):\n" << E[i] << std::endl;
	}

	Eigen::VectorXd motion = GENERATE_LINEAR(t, dt, v);

	// Run kalman filter loop
	Eigen::VectorXd x_p, u_t;
	Eigen::MatrixXd E_p;
	Eigen::VectorXd z_t(1,1);

	std::vector<double> x_estimate, v_estimate, E_norm;

	double *data_ptr = motion.data();
	std::vector<double> x_real(data_ptr, data_ptr + motion.size());

	for(int i=0; i < n; i++) {
		if(i >= 1) {
			x.push_back(x[i-1]);    // x_t initialized to x_t-1
			E.push_back(E[i-1]);    // likewise

			u_t = Eigen::VectorXd::Zero(2,1);
			z_t(0) = motion(i);

			filter->update(x[i],E[i],u_t,z_t);
		}
		x_estimate.push_back(x[i](0));
		v_estimate.push_back(x[i](1));
		E_norm.push_back(E[i].norm());
	}

	LOG4CXX_INFO(KalmanFilter::logger, "1D Linear Vel = \n" << *filter);

	// Plot
	try {
		Gnuplot g2;
		g2.plot_x(x_estimate,"x estimate");
		g2.plot_x(v_estimate,"v estimate");
		g2.plot_x(x_real,"x real");
		g2.set_smooth().plot_x(E_norm,"Cov norm");
		// g2.set_smooth("bezier").plot_x(y2,"bezier");
		// g2.unset_smooth();
		g2.set_style("lines");
		int tmp;
		// std::cin >> tmp;
	} catch (GnuplotException ge) {
        cout << ge.what() << endl;
    }
}
/* ********************************************************************************************* */
TEST(KALMAN, TEST_VOLTAGE) {
	KalmanFilter filter;
	Eigen::MatrixXd A(1,1), B(1,1), R(1,1);
	Eigen::MatrixXd C(1,1), Q(1,1);

	A(0,0) = 1;                 // State model
	B(0,0) = 0;                 // Command model
	R(0,0) = 0.00001;           // Covariance of posterior state: A_t*x_t-1 + B_t*u_t

	C(0,0) = 1;                 // Measurement model
	Q(0,0) = 1;                 // Measurement noise model

	filter.setTransitionModel(A, B, R);
	filter.setMeasurementModel(C, Q);

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(5.0,1.0); // normal_distribution(mean, std)

	std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > x;
	std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > E;
	x.push_back(Eigen::VectorXd::Zero(1,1));
	E.push_back(Eigen::MatrixXd::Identity(1,1));
	Eigen::VectorXd u_t = Eigen::VectorXd::Zero(1,1);
	Eigen::VectorXd z_t = Eigen::VectorXd::Zero(1,1);
	std::vector<double> x_est, E_est;
	std::vector<double> x_real;
	for(int i=0; i < 100; i++) {
		if(i > 0) {
			x.push_back(x[i-1]);    // x_t initialized to x_t-1
			E.push_back(E[i-1]);    // likewise

			z_t(0) = distribution(generator);

			filter.update(x[i],E[i],u_t,z_t);
		}
		x_est.push_back(x[i](0));
		E_est.push_back(E[i](0));
		x_real.push_back(z_t(0));
	}

	try {
		Gnuplot plot;
		plot.set_smooth().plot_x(x_est, "x est");
		plot.set_smooth().plot_x(x_real, "x real");
		plot.set_smooth().plot_x(E_est, "E est");
		// int tmp;
		// std::cin >> tmp;
	} catch (GnuplotException ge) {
        cout << ge.what() << endl;
    }
}
/* ********************************************************************************************* */
TEST(KALMAN, TEST_2D_BALL) {
	KalmanFilter filter;
	Eigen::MatrixXd A(6,6), B(6,6), R(6,6);
	Eigen::MatrixXd C(2,6), Q(2,2);

	double dt = 0.1;
	
	A.setZero();
	A.block<3,3>(0,0) <<
		1, dt, 1/2*dt*dt,
		0, 1, dt,
		0, 0, 1;
	A.block<3,3>(3,3) <<
		1, dt, 1/2*dt*dt,
		0, 1, dt,
		0, 0, 1;
	B.setZero();
	R.setIdentity();

	C.setZero();
	C(0,0) = 1;
	C(1,3) = 1;
	Q.setZero();

	// A(0,0) = 1;                 // State model
	// B(0,0) = 0;                 // Command model
	// R(0,0) = 0.00001;           // Covariance of posterior state: A_t*x_t-1 + B_t*u_t

	// C(0,0) = 1;                 // Measurement model
	// Q(0,0) = 1;                 // Measurement noise model

	filter.setTransitionModel(A, B, R);
	filter.setMeasurementModel(C, Q);

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(5.0,1.0); // normal_distribution(mean, std)

	std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd> > x;
	std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > E;
	x.push_back(Eigen::VectorXd::Zero(1,1));
	E.push_back(Eigen::MatrixXd::Identity(1,1));
	Eigen::VectorXd u_t = Eigen::VectorXd::Zero(1,1);
	Eigen::VectorXd z_t = Eigen::VectorXd::Zero(1,1);
	std::vector<double> x_est, E_est;
	std::vector<double> x_real;
	for(int i=0; i < 100; i++) {
		if(i > 0) {
			x.push_back(x[i-1]);    // x_t initialized to x_t-1
			E.push_back(E[i-1]);    // likewise

			z_t(0) = distribution(generator);

			filter.update(x[i],E[i],u_t,z_t);
		}
		x_est.push_back(x[i](0));
		E_est.push_back(E[i](0));
		x_real.push_back(z_t(0));
	}

	try {
		Gnuplot plot;
		plot.set_smooth().plot_x(x_est, "x est");
		plot.set_smooth().plot_x(x_real, "x real");
		plot.set_smooth().plot_x(E_est, "E est");
		// int tmp;
		// std::cin >> tmp;
	} catch (GnuplotException ge) {
        cout << ge.what() << endl;
    }
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
	// ::log4cxx::BasicConfigurator::configure();
	::log4cxx::xml::DOMConfigurator::configure(PROTO_ROOT_PATH "config/gtest.log4cxx.xml");
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */

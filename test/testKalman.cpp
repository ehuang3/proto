#include <iostream>
#include <cmath>
#include <gtest/gtest.h>
#include <kalman/kalman.h>
#include <gnuplot/gnuplot_i.hpp>

using namespace std;

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
		1, 0,
		0, dt;
	B.setZero();
	R << 
		0.5, 0,
		0, 0.5;
	C << 1, 0;
	Q << 0.5;

	filter_1D.setTransitionModel(A, B, R);
	filter_1D.setMeasurementModel(C, Q);

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

	std::cout << "x_p: \n" << x_p << std::endl;
	std::cout << "E_p: \n" << E_p << std::endl;

	std::cout << "filter->update()" << std::endl;
	filter->update(x_p, E_p, u_t, z_t);

	std::cout << "x_p: \n" << x_p << std::endl;
	std::cout << "E_p: \n" << E_p << std::endl;

	u_t(0) = 10.0;
	z_t(0) = 0.0;

	std::cout << "filter->update()" << std::endl;
	filter->update(x_p, E_p, u_t, z_t);

	std::cout << "x_p: \n" << x_p << std::endl;
	std::cout << "E_p: \n" << E_p << std::endl;

	u_t(0) = 10.0;
	z_t(0) = 0.0;

	std::cout << "filter->update()" << std::endl;
	filter->update(x_p, E_p, u_t, z_t);

	std::cout << "x_p: \n" << x_p << std::endl;
	std::cout << "E_p: \n" << E_p << std::endl;
}
/* ********************************************************************************************* */
TEST(KALMAN, TEST_1D_LINEAR_VEL) {
	double t = 5.0;
	double dt = 0.1;
	double v = 1;
	int n = t/dt;

	Eigen::VectorXd x(2,1);         // Mean and covariance at time t-1
	Eigen::MatrixXd E(2,2);         // 
	Eigen::VectorXd u_t(1), z_t(1); // Command and measurement at time t

	KalmanFilter* filter = GENERATE_FILTER_1D_VEL(dt);

	// initial conditions
	x << 0, 0;
	E << 1, 0, 0, 1;

	Eigen::VectorXd truth = GENERATE_LINEAR(t, dt, v);
	Eigen::VectroXd x_t(2,n);



	filter->update(x_p, E_p, u_t, z_t);
}
/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */

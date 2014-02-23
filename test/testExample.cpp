#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>

using namespace std;

/* ********************************************************************************************* */
TEST(EXAMPLE, FIRST_TEST) {
	cout << "Hello Test!" << endl;
}

/* ********************************************************************************************* */
TEST(EXAMPLE, SECOND_TEST) {
	Eigen::Vector4d v4 = Eigen::Vector4d::Zero();
	cout << v4 << endl;
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
/* ********************************************************************************************* */

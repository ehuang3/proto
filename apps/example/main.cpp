#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

int main() {
   Isometry3d X;
   X = Matrix4d::Identity();

   X.rotate(AngleAxisd(M_PI/3, Vector3d::UnitX()));
   cout << "X = \n" << X.matrix() << endl;

   X.translate(Vector3d(0, 10, -5));
   cout << "X = \n" << X.matrix() << endl;

   X.rotate(AngleAxisd(M_PI/3, Vector3d(.3, 0.5, 0.1).normalized()));
   cout << "X = \n" << X.matrix() << endl;
}

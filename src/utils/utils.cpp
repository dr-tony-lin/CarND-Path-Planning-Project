#include "utils.h"
#include <vector>

using namespace std;

double polyeval(Eigen::VectorXd coeffs, const double x) {
  double result = 0;
  for (int i = coeffs.size() - 1; i >= 0; i--) {
    result = result * x + coeffs[i];
  }
  return result;
}

double polyder(const Eigen::VectorXd coeffs, const double &x) {
  double result = 0;
  for (int i = coeffs.size() - 1; i >= 1; i--) {
    result = result * x + i * coeffs[i];
  }
  return result;
}

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

std::vector<double> unitv(double x, double y) {
  double len = length(x, y);
  if (len > 0) {
    return {x / len, y / len};
  }
  else {
    return {0, 0};
  }
}

/// Calculate the Jerk Minimizing Trajectory that connects the initial state
/// to the final state in time t.
std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double t)
{
    double t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
    Eigen::MatrixXd A(3,3);
    Eigen::VectorXd b(3);
     
    A << t3, t4, t5, 3*t2, 4*t3, 5*t4, 6*t, 12*t2, 20*t3;
    b << end[0] - (start[0] + start[1]*t + start[2]*t2/2),
         end[1] - (start[1] + start[2] * t),
         end[2] - start[2];
    Eigen::VectorXd alpha = A.colPivHouseholderQr().solve(b);
    return {start[0], start[1], start[2]/2, alpha(0), alpha(1), alpha(2)};
}

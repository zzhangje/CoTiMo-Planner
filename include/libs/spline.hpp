#ifndef SPLINE_HPP
#define SPLINE_HPP

#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/Sparse>

namespace spline {
void cubic(const Eigen::VectorXd& x, Eigen::VectorXd& a, Eigen::VectorXd& b,
           Eigen::VectorXd& c, Eigen::VectorXd& d) {
  int n = x.size() - 1;
  Eigen::SparseMatrix<double> A(n, n + 1), B(n, n + 1), C(n, n + 1), D(n, n + 1);

  A.insert(0, 0) = 2, A.insert(0, 1) = -3, A.insert(0, 2) = 1;
  B.insert(0, 0) = -3, B.insert(0, 1) = 4, B.insert(0, 2) = -1;

  for (int i = 1; i < n - 1; ++i) {
    A.insert(i, i) = 1, A.insert(i, i + 1) = -2, A.insert(i, i + 2) = 1;
    B.insert(i, i) = -1, B.insert(i, i + 1) = 2, B.insert(i, i + 2) = -1;
    C.insert(i, i) = -1, C.insert(i, i + 1) = 1;
  }

  A.insert(n - 1, n - 1) = 1, A.insert(n - 1, n) = -1;
  B.insert(n - 1, n - 1) = -1, B.insert(n - 1, n) = 1;
  C.insert(n - 1, n - 1) = -1, C.insert(n - 1, n) = 1;

  a = A * x;
  b = B * x;
  c = C * x;
  d = x.segment(0, n);
  return;
}
}  // namespace spline
#endif
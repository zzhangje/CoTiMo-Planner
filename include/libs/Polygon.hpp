#ifndef POLYGON_HPP
#define POLYGON_HPP

#include <Eigen/Eigen>

#include "log.hpp"
#include "sdqp.hpp"

namespace nextinnovation {
// A 2D polygon class
class Polygon {
 private:
  std::vector<Eigen::Vector2d> points;

  // Ax <= b
  Eigen::MatrixX2d A;
  Eigen::VectorXd b;

  // the center and orientation of the polygon
  Eigen::Vector2d p;
  double q;

  // SDQP const matrix
  Eigen::Matrix4d Q;
  Eigen::Vector4d c;

  /**
   * @brief Convert the polygon to a linear system of inequalities, Ax <= b
   */
  void convert() {
    int n = this->points.size();
    this->A = Eigen::MatrixX2d::Zero(n, 2);
    this->b = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; i++) {
      Eigen::Vector2d p1 = this->points[i];
      Eigen::Vector2d p2 = this->points[(i + 1) % n];
      Eigen::Vector2d v = p2 - p1;
      Eigen::Vector2d n = Eigen::Vector2d(-v.y(), v.x());
      this->A.row(i) = n;
      this->b(i) = n.dot(p1);
    }
  }

 public:
  /**
   * @param points
   * @param isCCW If the points are in counter-clockwise order
   */
  Polygon(const std::vector<Eigen::Vector2d>& points, bool isCCW = true) {
    this->points =
        isCCW ? std::vector<Eigen::Vector2d>(points.rbegin(), points.rend())
              : points;
    this->p = Eigen::Vector2d(0, 0);
    this->q = 0;
    this->convert();

    // precompute the SDQP const matrix
    Q << 2, 0, -2, 0, 0, 2, 0, -2, -2, 0, 2, 0, 0, -2, 0, 2;
    c << 0, 0, 0, 0;
  }

  Polygon transform(Eigen::Vector2d dp, double dq) {
    std::vector<Eigen::Vector2d> new_points;
    double cos_theta = cos(dq);
    double sin_theta = sin(dq);
    for (Eigen::Vector2d p : this->points) {
      new_points.push_back(Eigen::Vector2d(
          this->p.x() + cos_theta * p.x() - sin_theta * p.y() + dp.x(),
          this->p.y() + sin_theta * p.x() + cos_theta * p.y() + dp.y()));
    }
    return Polygon(new_points);
  }

  bool isPointInside(Eigen::Vector2d p) {
    return (this->A * p - this->b).maxCoeff() <= 0;
  }

  bool isPolygonIntersect(Polygon poly) {
    // check if the two polygons intersect
    for (int i = 0; i < this->points.size(); i++) {
      Eigen::Vector2d p1 = this->points[i];
      Eigen::Vector2d p2 = this->points[(i + 1) % this->points.size()];
      for (int j = 0; j < poly.points.size(); j++) {
        Eigen::Vector2d q1 = poly.points[j];
        Eigen::Vector2d q2 = poly.points[(j + 1) % poly.points.size()];
        double d = (p1.x() - p2.x()) * (q1.y() - q2.y()) -
                   (p1.y() - p2.y()) * (q1.x() - q2.x());
        if (d == 0) {
          continue;
        }
        double t = ((p1.x() - q1.x()) * (q1.y() - q2.y()) -
                    (p1.y() - q1.y()) * (q1.x() - q2.x())) /
                   d;
        double u = -((p1.x() - p2.x()) * (p1.y() - q1.y()) -
                     (p1.y() - p2.y()) * (p1.x() - q1.x())) /
                   d;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
          return true;
        }
      }
    }
    // check if one polygon is inside the other
    bool inside = true;
    for (Eigen::Vector2d p : this->points) {
      if (!poly.isPointInside(p)) {
        inside = false;
        break;
      }
    }
    if (inside) {
      return true;
    }
    for (Eigen::Vector2d p : poly.points) {
      if (!this->isPointInside(p)) {
        return false;
      }
    }
    inside = true;
    for (Eigen::Vector2d p : poly.points) {
      if (!this->isPointInside(p)) {
        inside = false;
        break;
      }
    }
    return inside;
  }

  double distanceToPoint(Eigen::Vector2d p) {
    if (this->isPointInside(p)) {
      double result = INFINITY;
      for (int i = 0; i < this->points.size(); i++) {
        result = std::min(result,
                          (b(i) - A.row(i).dot(p)) / (A.row(i).norm() + 1e-20));
      }
      return -result;
    }
    if (A.rows() == 1) {
      return (A.row(0).dot(p) - b(0)) / (A.row(0).norm() + 1e-20);
    }
    Eigen::Vector2d x = p;
    sdqp::sdqp<2>(2 * Eigen::MatrixXd::Identity(2, 2), -2 * p, A, b, x);
    return (p - x).norm();
  }

  double distanceToPolygon(Polygon poly) {
    // SDQP problem:
    // min  0.5 x' Q x + c' x
    // s.t. A x <= b
    // Q should be PD, but PSD is enough here
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    int n1 = this->points.size();
    int n2 = poly.points.size();
    if (this->isPolygonIntersect(poly)) {
      // ND problem cannot be solved properly
      // we use an approximate solution
      // this may be better than the one in report in fact
      double result = INFINITY;
      for (int i = 0; i < n1; i++) {
        if (poly.isPointInside(this->points[i])) {
          result = std::min(result, poly.distanceToPoint(this->points[i]));
        }
      }
      for (int i = 0; i < n2; i++) {
        if (this->isPointInside(poly.points[i])) {
          result = std::min(result, this->distanceToPoint(poly.points[i]));
        }
      }
      return result;
    }
    Eigen::MatrixX4d c_A = Eigen::MatrixX4d::Zero(n1 + n2, 4);
    Eigen::VectorXd c_b = Eigen::VectorXd::Zero(n1 + n2);
    for (int i = 0; i < n1; i++) {
      c_A(i, 0) = this->A(i, 0);
      c_A(i, 1) = this->A(i, 1);
      c_b(i) = this->b(i);
    }
    for (int i = 0; i < n2; i++) {
      c_A(i + n1, 2) = poly.A(i, 0);
      c_A(i + n1, 3) = poly.A(i, 1);
      c_b(i + n1) = poly.b(i);
    }
    sdqp::sdqp<4>(Q, c, c_A, c_b, x);
    return (x.head(2) - x.tail(2)).norm();
  }

  void getPoints(std::vector<Eigen::Vector2d>& points) {
    points = this->points;
  }
  std::vector<Eigen::Vector2d> getPoints() { return this->points; }

  void getLinearSystem(Eigen::MatrixX2d& A, Eigen::VectorXd& b) {
    A = this->A;
    b = this->b;
  }
};
};  // namespace nextinnovation

#endif  // POLYGON_HPP
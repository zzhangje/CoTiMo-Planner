#ifndef UTILS_HPP
#define UTILS_HPP

#include <eigen3/Eigen/Eigen>

std::vector<Eigen::VectorXd> socProjections(
    const std::vector<Eigen::VectorXd>& v) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < v.size(); ++i) {
    res.push_back(socProjection(v[i]));
  }
  return res;
}

std::vector<double> norm(std::vector<Eigen::VectorXd> vector) {
  std::vector<double> res;
  for (int i = 0; i < vector.size(); ++i) {
    res.push_back(vector[i].norm());
  }
  return res;
}

double sum(std::vector<double> vec) {
  double res = 0;
  for (int i = 0; i < vec.size(); ++i) {
    res += vec[i];
  }
  return res;
}

Eigen::VectorXd sum(std::vector<Eigen::VectorXd> vector) {
  if (vector.size() < 1) {
    return Eigen::VectorXd::Zero(0);
  }
  Eigen::VectorXd res = vector[0];
  for (int i = 1; i < vector.size(); ++i) {
    res += vector[i];
  }
  return res;
}

std::vector<Eigen::VectorXd> operator/(const std::vector<Eigen::VectorXd>& vectors, const double num) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i] / num);
  }
  return res;
}

std::vector<Eigen::VectorXd> operator*(const std::vector<Eigen::VectorXd>& vectors, const double num) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i] * num);
  }
  return res;
}

std::vector<Eigen::VectorXd> operator*(const double num, const std::vector<Eigen::VectorXd>& vectors) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i] * num);
  }
  return res;
}

std::vector<Eigen::MatrixXd> operator*(const std::vector<Eigen::MatrixXd>& matrices, const double num) {
  std::vector<Eigen::MatrixXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(matrices[i] * num);
  }
  return res;
}

std::vector<Eigen::MatrixXd> operator*(const double num, const std::vector<Eigen::MatrixXd>& matrices) {
  std::vector<Eigen::MatrixXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(matrices[i] * num);
  }
  return res;
}

std::vector<Eigen::VectorXd> operator*(const std::vector<double>& nums, const std::vector<Eigen::VectorXd>& vectors) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i] * nums[i]);
  }
  return res;
}

std::vector<Eigen::MatrixXd> operator*(const std::vector<double>& nums, const std::vector<Eigen::MatrixXd>& matrices) {
  std::vector<Eigen::MatrixXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(matrices[i] * nums[i]);
  }
  return res;
}

std::vector<Eigen::MatrixXd> operator*(const std::vector<Eigen::MatrixXd>& matrices, const std::vector<double>& nums) {
  std::vector<Eigen::MatrixXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(matrices[i] * nums[i]);
  }
  return res;
}

std::vector<Eigen::RowVectorXd> operator*(const std::vector<Eigen::RowVectorXd>& vectors, const double num) {
  std::vector<Eigen::RowVectorXd> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i] * num);
  }
  return res;
}

std::vector<Eigen::RowVectorXd> operator*(const double num, const std::vector<Eigen::RowVectorXd>& vectors) {
  std::vector<Eigen::RowVectorXd> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i] * num);
  }
  return res;
}

std::vector<Eigen::VectorXd> operator*(const std::vector<Eigen::MatrixXd>& matrices, const Eigen::VectorXd vector) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(matrices[i] * vector);
  }
  return res;
}

std::vector<Eigen::VectorXd> operator*(const std::vector<Eigen::MatrixXd>& matrices, const std::vector<Eigen::VectorXd> vectors) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(matrices[i] * vectors[i]);
  }
  return res;
}

std::vector<Eigen::RowVectorXd> operator*(const Eigen::RowVectorXd vector, const std::vector<Eigen::MatrixXd>& matrices) {
  std::vector<Eigen::RowVectorXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(vector * matrices[i]);
  }
  return res;
}

std::vector<double> operator*(const std::vector<Eigen::RowVectorXd>& vectors, const Eigen::VectorXd vector) {
  std::vector<double> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i].dot(vector));
  }
  return res;
}

std::vector<Eigen::MatrixXd> operator*(const std::vector<Eigen::MatrixXd>& matricesA, const std::vector<Eigen::MatrixXd>& matricesB) {
  std::vector<Eigen::MatrixXd> res;
  for (int i = 0; i < matricesA[i].size(); ++i) {
    res.push_back(matricesA[i] * matricesB[i]);
  }
  return res;
}

std::vector<double> operator-(const std::vector<double>& vecA, const std::vector<double>& vecB) {
  std::vector<double> res;
  for (int i = 0; i < vecA.size(); ++i) {
    res.push_back(vecA[i] - vecB[i]);
  }
  return res;
}

std::vector<Eigen::VectorXd> operator-(const std::vector<Eigen::VectorXd>& vectorsA, const std::vector<Eigen::VectorXd>& vectorsB) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < vectorsA.size(); ++i) {
    res.push_back(vectorsA[i] - vectorsB[i]);
  }
  return res;
}

std::vector<Eigen::VectorXd> operator+(const std::vector<Eigen::VectorXd>& vectorsA, const std::vector<Eigen::VectorXd>& vectorsB) {
  std::vector<Eigen::VectorXd> res;
  for (int i = 0; i < vectorsA.size(); ++i) {
    res.push_back(vectorsA[i] + vectorsB[i]);
  }
  return res;
}

std::vector<Eigen::MatrixXd> transpose(const std::vector<Eigen::MatrixXd>& matrices) {
  std::vector<Eigen::MatrixXd> res;
  for (int i = 0; i < matrices.size(); ++i) {
    res.push_back(matrices[i].transpose());
  }
  return res;
}

std::vector<Eigen::RowVectorXd> transpose(const std::vector<Eigen::VectorXd>& vectors) {
  std::vector<Eigen::RowVectorXd> res;
  for (int i = 0; i < vectors.size(); ++i) {
    res.push_back(vectors[i].transpose());
  }
  return res;
}

std::vector<double> squaredNorm(std::vector<Eigen::VectorXd> vector) {
  std::vector<double> res;
  for (int i = 0; i < vector.size(); ++i) {
    res.push_back(vector[i].squaredNorm());
  }
  return res;
}

double squaredNorm(std::vector<double> vec) {
  double res = 0;
  for (int i = 0; i < vec.size(); ++i) {
    res += vec[i] * vec[i];
  }
  return res;
}

#endif  // UTILS_HPP
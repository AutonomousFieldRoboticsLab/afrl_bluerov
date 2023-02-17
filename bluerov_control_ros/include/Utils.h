#pragma once

#include <Eigen/Core>
#include <cmath>
#include <iostream>

namespace utils {

template <typename T>
T constrainValue(const T value, const T min, const T max) {
  if (std::isnan(value)) return (min + max) / 2;
  return (value < min) ? min : ((value > max) ? max : value);
}

auto constrainFloat = [](const float value, const float min, const float max) {
  return constrainValue<float>(value, min, max);
};

auto constrainInt = [](const int value, const int min, const int max) {
  return constrainValue<int>(value, min, max);
};

double ccwAngleBetweenVectors(double x1, double y1, double x2, double y2);
double angleErrorRadians(double target_angle, double current_angle);
Eigen::Vector3d getAngleError(const Eigen::Vector3d& target_rpy,
                              const Eigen::Vector3d& current_rpy);

template <typename T>
void normalizeVector(std::vector<T>& input_vector) {
  T maximum = *std::max_element(input_vector.begin(), input_vector.end());
  for (T& num : input_vector) {
    num = num / maximum;
  }
};

template <typename T>
void printVector(std::vector<T>& input_vector) {
  for (auto num : input_vector) {
    std::cout << num << "\t";
  }
  std::cout << std::endl;
};

}  // namespace utils

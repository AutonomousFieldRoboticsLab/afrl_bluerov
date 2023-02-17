#include "Utils.h"

#include <cmath>

double utils::ccwAngleBetweenVectors(double x1, double y1, double x2, double y2) {
  // from
  // https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors
  double dot = x1 * x2 + y1 * y2;  // dot product between [x1, y1] and [x2, y2]
  double det = x1 * y2 - y1 * x2;  // determinant
  double angle = atan2(det, dot);  // atan2(y, x) or atan2(sin, cos)

  if (angle < 0) {
    return (2.0 * M_PI) + angle;
  }
  return angle;
}

double utils::angleErrorRadians(double target_angle, double from_angle) {
  // smallest distance in radians b / t ccw and cw angles
  double x1 = cos(from_angle);
  double y1 = sin(from_angle);

  double x2 = cos(target_angle);
  double y2 = sin(target_angle);

  double ccw_distance = utils::ccwAngleBetweenVectors(x1, y1, x2, y2);

  if (ccw_distance > M_PI) {
    // clockwise is faster in this case
    double cw_distance = -1 * ((2.0 * M_PI) - ccw_distance);
    return cw_distance;
  }
  return ccw_distance;
}

Eigen::Vector3d utils::getAngleError(const Eigen::Vector3d& target_rpy,
                                     const Eigen::Vector3d& current_rpy) {
  double roll_error = utils::angleErrorRadians(target_rpy[0], current_rpy[0]);
  double pitch_error = utils::angleErrorRadians(target_rpy[1], current_rpy[1]);
  double yaw_error = utils::angleErrorRadians(target_rpy[2], current_rpy[2]);

  return Eigen::Vector3d(roll_error, pitch_error, yaw_error);
}


#include "Primitives.h"

MotionPrimitive::MotionPrimitive() {
  speed_ = 1.0;
  feeback_method_ = FeedbackMethod::ATTITUDE_WITH_DEPTH;
}

MotionPrimitive::MotionPrimitive(float speed, FeedbackMethod feedback_method) {
  speed_ = speed;
  feeback_method_ = feedback_method;
}

void MotionPrimitive::setAttitude(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  // attitude_ = attitude;
}
void MotionPrimitive::setDepth(const float depth) { depth_ = depth; }
// void MotionPrimitive::setAttitudeAndDepth(const Eigen::Vector3f& attitude, const float depth) {
//   attitude_ = attitude;
//   depth_ = depth;
// }

void MotionPrimitive::setPose(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
  // attitude_ = attitude;
  // position_ = position;
}

Trasect::Trasect() : MotionPrimitive(), length_(5.0), duration_(5.0) {}

Trasect::Trasect(float length, float duration)
    : MotionPrimitive(), length_(length), duration_(duration) {}

Trasect::Trasect(float length, float duration, float speed, FeedbackMethod feedback_method)
    : MotionPrimitive(speed, feedback_method), length_(length), duration_(duration) {}

Trasect::~Trasect() {}

bool Trasect::execute() { return false; }

Square::Square() : MotionPrimitive(), length_(5.0), duration_(5.0) {}

Square::Square(float length, float duration)
    : MotionPrimitive(), length_(length), duration_(duration) {}

Square::Square(float length, float duration, float speed, FeedbackMethod feedback_method)
    : MotionPrimitive(speed, feedback_method), length_(length), duration_(duration) {}

Square::~Square() {}

bool Square::execute() { return false; }

LawnMower::LawnMower()
    : MotionPrimitive(),
      long_strip_duration_(10.0),
      short_strip_duration_(3.0),
      strip_length_(10.0),
      strip_width_(3.0) {}

LawnMower::LawnMower(float long_strip_duration,
                     float short_strip_duration,
                     float strip_length,
                     float strip_width)
    : MotionPrimitive(),
      long_strip_duration_(long_strip_duration),
      short_strip_duration_(short_strip_duration),
      strip_length_(strip_length),
      strip_width_(strip_width) {}

LawnMower::LawnMower(float long_strip_duration,
                     float short_strip_duration,
                     float strip_length,
                     float strip_width,
                     float speed,
                     FeedbackMethod feedback_method)
    : MotionPrimitive(speed, feedback_method),
      long_strip_duration_(long_strip_duration),
      short_strip_duration_(short_strip_duration),
      strip_length_(strip_length),
      strip_width_(strip_width) {}

LawnMower::~LawnMower() {}

bool LawnMower::execute() { return false; }

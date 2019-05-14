// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework2/rotation/rotation.h"

namespace homework2 {

const double EPS = 1e-9;

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation) {
  Eigen::Vector3d ret;
  double cosy = sqrt(rotation(2, 1) * rotation(2, 1) + rotation(2, 2) * rotation(2, 2));
  if (cosy < EPS) {
    ret[0] = 0;
    ret[1] = atan2(-rotation(2, 0), cosy);
    ret[2] = atan2(-rotation(1, 2), rotation(1, 1));
  }
  else {
    ret[0] = atan2(rotation(1, 0), rotation(0, 0));
    ret[1] = atan2(-rotation(2, 0), cosy);
    ret[2] = atan2(rotation(2, 1), rotation(2, 2));
  }
  return ret;
}

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation) {
  double costheta = (rotation(0, 0) + rotation(1, 1) + rotation(2, 2) - 1) / 2;
  Eigen::Vector3d axis;
  if (costheta > 1 - EPS) {
    axis[0] = 1;
    axis[1] = 0;
    axis[2] = 0;
  }
  else {
    double sintheta = sqrt(1 - costheta * costheta);
    axis[0] = sqrt((rotation(0, 0) - costheta) / (1 - costheta));
    axis[1] = sqrt((rotation(1, 1) - costheta) / (1 - costheta));
    axis[2] = sqrt((rotation(2, 2) - costheta) / (1 - costheta));
    double min_error = 1e100;
    int min_error_sign = 0;
    for (int i = 0; i < 8; i++) {
      double x = i & 1 ? -axis[0] : axis[0];
      double y = i & 2 ? -axis[1] : axis[1];
      double z = i & 4 ? -axis[2] : axis[2];
      double now_error = std::abs(z * sintheta + x * y * (1 - costheta) - rotation(1, 0)) +
          std::abs(-y * sintheta + x * z * (1 - costheta) - rotation(2, 0)) + std::abs(x *
          sintheta + y * z * (1 - costheta) - rotation(2, 1));
      if (now_error < min_error) {
        min_error = now_error;
        min_error_sign = i;
      }
    }
    if (min_error_sign & 1)
      axis[0] = -axis[0];
    if (min_error_sign & 2)
      axis[1] = -axis[1];
    if (min_error_sign & 4)
      axis[2] = -axis[2];
  }
  return Eigen::AngleAxisd(acos(costheta), axis);
}
}  // namespace homework2

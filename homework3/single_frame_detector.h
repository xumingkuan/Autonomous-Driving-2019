// Copyright @2019 Pony AI Inc. All rights reserved.

#pragma once

#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "common/utils/math/vec2d.h"
#include "homework2/pointcloud.h"
#include "homework3/obstacle.h"

class SingleFrameDetector {
 public:
  SingleFrameDetector();

  void GetGroundAndObstacles(
      const PointCloud& point_cloud,
      std::vector<Eigen::Vector3d>* ground_points,
      std::vector<Obstacle>* obstacles);

 private:
  std::deque<std::pair<Eigen::MatrixXd, std::pair<int, int> > > prev_grids_;
  int prev_max_num_frame_;
};

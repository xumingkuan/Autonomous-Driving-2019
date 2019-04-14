// Copyright @2019 Pony AI Inc. All rights reserved.

#include <algorithm>
#include <numeric>
#include <stack>

#include "common/utils/math/vec2d.h"
#include "homework3/single_frame_detector.h"

#include "glog/logging.h"

namespace {

bool CompareVec2d(const math::Vec2d& a, const math::Vec2d& b) {
  return a.x == b.x ? a.y < b.y : a.x < b.x;
}

std::vector<math::Vec2d> ComputeConvexHull(std::vector<math::Vec2d> points) {
  if (points.size() <= 2) {
    return points;
  }
  std::sort(points.begin(), points.end(), CompareVec2d);
  std::vector<math::Vec2d> up, down;
  for (math::Vec2d point : points) {
    while (up.size() >= 2 && math::OuterProd(up.back(), point, up[up.size() - 2]) >= 0) {
      up.pop_back();
    }
    while (down.size() >= 2 && math::OuterProd(down.back(), point, down[down.size() - 2]) <= 0) {
      down.pop_back();
    }
    up.push_back(point);
    down.push_back(point);
  }
  down.pop_back();
  std::reverse(up.begin(), up.end());
  if (down.front() == up.back()) {
    up.pop_back();
  }
  down.insert(down.end(), up.begin(), up.end());
  return down;
}

Obstacle BuildObstacleFromPoints(std::vector<Eigen::Vector3d> points) {
  Obstacle obstacle;
  std::vector<math::Vec2d> projected_points;
  for (Eigen::Vector3d point : points) {
    obstacle.floor = std::min(obstacle.floor, point.z());
    obstacle.ceiling = std::max(obstacle.ceiling, point.z());
    projected_points.emplace_back(math::Vec2d(point.x(), point.y()));
  }
  obstacle.polygon = ComputeConvexHull(projected_points);
  return obstacle;
}

}  // namespace

SingleFrameDetector::SingleFrameDetector() {
  // TODO(you): optional, if you need to do anything to initialize the detector, you can do it here.
  // For example, you can load offline computed ground information here.
  prev_max_num_frame_ = 7;
}

void SingleFrameDetector::GetGroundAndObstacles(
    const PointCloud& point_cloud,
    std::vector<Eigen::Vector3d>* ground_points,
    std::vector<Obstacle>* obstacles) {
  CHECK(ground_points != nullptr);
  CHECK(obstacles != nullptr);
  const int point_cloud_size = static_cast<int>(point_cloud.points.size());
  LOG(ERROR) << "Start GetGroundAndObstacles with " << point_cloud_size << " points.";

  // Detect the ground and put all the points inside the ground_points:

  // Transform the coordinates into global coordinate system.
  std::vector<Eigen::Vector3d> transformed_point_cloud;
  for (const Eigen::Vector3d& point : point_cloud.points) {
    transformed_point_cloud.emplace_back(point_cloud.rotation * point + point_cloud.translation);
  }

  // Project them into a 2D grid.
  std::vector<int> point_cloud_x, point_cloud_y;
  const double grid_length = 0.5;
  for (const Eigen::Vector3d& point : transformed_point_cloud) {
    point_cloud_x.emplace_back(std::lround(point.x() / grid_length));
    point_cloud_y.emplace_back(std::lround(point.y() / grid_length));
  }
  auto point_cloud_minmax_x = std::minmax_element(point_cloud_x.begin(), point_cloud_x.end());
  auto point_cloud_minmax_y = std::minmax_element(point_cloud_y.begin(), point_cloud_y.end());
  int now_x_offset = *point_cloud_minmax_x.first;
  int now_y_offset = *point_cloud_minmax_y.first;
  int now_x_size = *point_cloud_minmax_x.second - now_x_offset + 1;
  int now_y_size = *point_cloud_minmax_y.second - now_y_offset + 1;
  Eigen::MatrixXd now_grid = Eigen::MatrixXd::Constant(now_x_size, now_y_size,
      std::numeric_limits<double>::infinity());
  for (int i = 0; i < point_cloud_size; i++) {
    double& now_grid_position = now_grid(point_cloud_x[i] - now_x_offset,
                                         point_cloud_y[i] - now_y_offset);
    now_grid_position = std::min(now_grid_position, transformed_point_cloud[i].z());
  }
  prev_grids_.emplace_back(std::make_pair(now_grid, std::make_pair(now_x_offset, now_y_offset)));

  // Use previous frames to update the grid.
  for (auto it = prev_grids_.begin(); it != --prev_grids_.end(); it++) {
    const Eigen::MatrixXd& prev_grid = it->first;
    int delta_x_offset = it->second.first - now_x_offset;
    int delta_y_offset = it->second.second - now_y_offset;
    for (int x = std::max(0, -delta_x_offset);
    x < std::min(static_cast<int>(prev_grid.rows()), now_x_size - delta_x_offset); x++) {
      for (int y = std::max(0, -delta_y_offset);
      y < std::min(static_cast<int>(prev_grid.cols()), now_y_size - delta_y_offset); y++) {
        now_grid(x + delta_x_offset, y + delta_y_offset) =
            std::min(now_grid(x + delta_x_offset, y + delta_y_offset), prev_grid(x, y));
      }
    }
  }

  // Put all the detected ground points inside the ground_points.
  // And prepare for flood fill in the next part.
  Eigen::MatrixXi obstacle_grid = Eigen::MatrixXi::Zero(now_x_size, now_y_size);
  std::vector<bool> is_ground(point_cloud_size, false);
  std::stack<std::pair<int, int> > occupied_grid;
  CHECK(prev_max_num_frame_ >= 0);
  const double ground_point_threshold = 0.2 + 0.1 * prev_grids_.size() / (prev_max_num_frame_ + 1);
  if (prev_grids_.size() > prev_max_num_frame_) {
    prev_grids_.pop_front();
  }
  for (int i = 0; i < point_cloud_size; i++) {
    if (transformed_point_cloud[i][2] <=
        now_grid(point_cloud_x[i] - now_x_offset, point_cloud_y[i] - now_y_offset)
            + ground_point_threshold) {
      ground_points->push_back(transformed_point_cloud[i]);
      is_ground[i] = true;
    }
    else {
      if(obstacle_grid(point_cloud_x[i] - now_x_offset, point_cloud_y[i] - now_y_offset) == 0) {
        obstacle_grid(point_cloud_x[i] - now_x_offset, point_cloud_y[i] - now_y_offset) = -1;
        occupied_grid.push(std::make_pair(point_cloud_x[i] - now_x_offset,
            point_cloud_y[i] - now_y_offset));
      }
    }
  }
  LOG(ERROR) << "Ground points have been detected using a " << now_x_size << " * " << now_y_size
             << " matrix.";

  // Run flood fill.
  int num_obstacle = 0;
  const int search_radius = 1;
  while (!occupied_grid.empty()) {
    int x = occupied_grid.top().first;
    int y = occupied_grid.top().second;
    occupied_grid.pop();
    if (obstacle_grid(x, y) == -1) {
      num_obstacle++;
      obstacle_grid(x, y) = num_obstacle;
    }
    for (int i = std::max(0, x - search_radius);
    i <= std::min(now_x_size - 1, x + search_radius); i++) {
      for (int j = std::max(0, y - search_radius);
      j <= std::min(now_y_size - 1, y + search_radius); j++) {
        if (obstacle_grid(i, j) == -1) {
          obstacle_grid(i, j) = obstacle_grid(x, y);
          occupied_grid.push(std::make_pair(i, j));
        }
      }
    }
  }

  // Get the points for the obstacles.
  std::vector<std::vector<Eigen::Vector3d> > obstacle_points(num_obstacle);
  for (int i = 0; i < point_cloud_size; i++) {
    if (!is_ground[i]) {
      obstacle_points[
          obstacle_grid(point_cloud_x[i] - now_x_offset, point_cloud_y[i] - now_y_offset) - 1
      ].emplace_back(transformed_point_cloud[i]);
    }
  }

  // Get the polygons for the obstacles.
  const int min_num_points_of_obstacle = 10;
  for (int i = 0; i < num_obstacle; i++) {
    if (obstacle_points[i].size() >= min_num_points_of_obstacle) {
      obstacles->emplace_back(BuildObstacleFromPoints(obstacle_points[i]));
    }
  }
  LOG(ERROR) << obstacles->size() << " of " << num_obstacle << " possible obstacles detected.";
}

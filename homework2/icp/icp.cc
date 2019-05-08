// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework2/icp/icp.h"

#include <Eigen/SVD>
#include <Eigen/Dense>

#include "common/utils/math/math_utils.h"
#include "common/utils/math/transform/transform.h"

Icp::Icp(const PointCloud& src_pc, const PointCloud& target_pc) {
  src_points_ = Eigen::MatrixXd::Zero(3, src_pc.points.size());
  for (int i = 0; i < src_pc.points.size(); ++i) {
    src_points_.col(i) = src_pc.points[i];
  }
  transformed_src_points_ = src_points_;

  target_points_ = Eigen::MatrixXd::Zero(3, target_pc.points.size());
  for (int i = 0; i < target_pc.points.size(); ++i) {
    target_points_.col(i) = target_pc.points[i];
  }
  target_pc_knn_ = std::make_unique<KdTree>(&target_points_);
}

bool Icp::RunIteration() {
  int iter = 0;
  while (iter < max_iteration_) {
    int num_correspondence = FindCorrespondence(max_correspondence_distance_);
    if (num_correspondence == 0) {
      LOG(WARNING) << "Fail to find any correspondences, exit.";
      return false;
    }
    const double transform_delta = EstimatePose();
    if (transform_delta < 0.0) {
      LOG(WARNING) << "ICP fail to converge, exit. transform_delta < 0.0";
      return false;
    }
    if (transform_delta < kDefaultMinTransformDelta) {
      const Eigen::Quaterniond quaternion(rotation_);
      LOG(INFO) << "ICP converges at iter: " << iter
                << ". T: " << translation_.transpose()
                << ", R/P/Y: " << math::RadianToDegree(math::transform::GetRoll(quaternion))
                << "," << math::RadianToDegree(math::transform::GetPitch(quaternion))
                << "," << math::RadianToDegree(math::transform::GetYaw(quaternion));
      return true;
    }
    ++iter;
  }
  const Eigen::Quaterniond quaternion(rotation_);
  LOG(WARNING) << "ICP fail to converge within the required iteration. "
               << ". T: " << translation_.transpose()
               << ", R/P/Y: " << math::RadianToDegree(math::transform::GetRoll(quaternion))
               << "," << math::RadianToDegree(math::transform::GetPitch(quaternion))
               << "," << math::RadianToDegree(math::transform::GetYaw(quaternion));
  return false;
}

/*
 *
 * Implement this function. Try to find the correspondences by using the Kd-Tree.
 *
 */
int Icp::FindCorrespondence(double max_correspondence_distance) {
  correspondences_.clear();
  return correspondences_.size();
}

/*
 *
 * Implement this function. Estimate R|T given correspondences.
 *
 */
double Icp::EstimatePose() {
  const int active_pt_num = correspondences_.size();
  if (active_pt_num < min_num_correspondence_) {
    return -1.0;
  }

  // 1. Construct source/target point matrix from select correspondences;

  // 2. Find the centroid and demean source/target point matrix;

  // 3. Follow the proof in handout and estimate R|T for this iteration;
  Eigen::Matrix3d rotation_cur_iter = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_cur_iter = Eigen::Vector3d::Zero();

  // 4. Transform source pointcloud by using estimated R|T

  // 5. Update accumulated rotation_ and translation_.


  return std::max((rotation_cur_iter - Eigen::Matrix3d::Identity()).norm(),
                  translation_cur_iter.norm());
}

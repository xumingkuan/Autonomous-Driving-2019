// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework2/icp/icp.h"

#include <Eigen/SVD>
#include <Eigen/Dense>

#include "common/utils/math/math_utils.h"
#include "common/utils/math/transform/transform.h"

class MatrixXd;
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
  // LOG(ERROR) << "Start finding correspondences in " << transformed_src_points_.size()
  //            << " source points and " << target_points_.size() << " target points." << std::endl;
  correspondences_.clear();
  std::vector<int> k_indices;
  std::vector<double> k_sqr_distances;
  for (int i = 0; i < transformed_src_points_.cols(); i++) {
    const auto& point = transformed_src_points_.col(i);
    int num_found = target_pc_knn_->RadiusSearch(point, max_correspondence_distance, &k_indices,
        &k_sqr_distances);
    if (num_found > 0)
      correspondences_.emplace_back(i, k_indices.front(), k_sqr_distances.front());
  }
  // LOG(ERROR) << "Found " << correspondences_.size() << " correspondences." << std::endl;
  return static_cast<int>(correspondences_.size());
}

/*
 *
 * Implement this function. Estimate R|T given correspondences.
 *
 */
double Icp::EstimatePose() {
  const int active_pt_num = static_cast<int>(correspondences_.size());
  if (active_pt_num < min_num_correspondence_) {
    return -1.0;
  }

  // 1. Construct source/target point matrix from select correspondences;
  Eigen::MatrixXd p(3, active_pt_num);
  Eigen::MatrixXd q(3, active_pt_num);
  for (int i = 0; i < active_pt_num; i++) {
    p.col(i) = transformed_src_points_.col(i);
    q.col(i) = target_points_.col(i);
  }

  // 2. Find the centroid and demean source/target point matrix;
  Eigen::Vector3d p_centroid(0, 0, 0);
  Eigen::Vector3d q_centroid(0, 0, 0);
  for (int i = 0; i < active_pt_num; i++) {
    p_centroid += p.col(i);
    q_centroid += q.col(i);
  }
  p_centroid /= active_pt_num;
  q_centroid /= active_pt_num;
  for (int i = 0; i < active_pt_num; i++) {
    p.col(i) -= p_centroid;
    q.col(i) -= q_centroid;
  }

  // 3. Follow the proof in handout and estimate R|T for this iteration;
  Eigen::Matrix3d rotation_cur_iter = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation_cur_iter = Eigen::Vector3d::Zero();
  Eigen::Matrix3d s = p * q.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(s, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d w = Eigen::Matrix3d::Identity();
  w(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  rotation_cur_iter = svd.matrixV() * w * svd.matrixU().transpose();
  translation_cur_iter = q_centroid - rotation_cur_iter * p_centroid;
  // LOG(ERROR) << std::endl << rotation_cur_iter << std::endl << translation_cur_iter << std::endl;

  // 4. Transform source pointcloud by using estimated R|T
  for (int i = 0; i < transformed_src_points_.cols(); i++) {
    transformed_src_points_.col(i) = rotation_cur_iter * transformed_src_points_.col(i) +
        translation_cur_iter;
  }

  // 5. Update accumulated rotation_ and translation_.
  rotation_ = rotation_cur_iter * rotation_;
  translation_ += translation_cur_iter;

  return std::max((rotation_cur_iter - Eigen::Matrix3d::Identity()).norm(),
                  translation_cur_iter.norm());
}

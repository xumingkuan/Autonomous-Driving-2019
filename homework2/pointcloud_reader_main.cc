// Copyright @2018 Pony AI Inc. All rights reserved.

#include <iostream>

#include "homework2/pointcloud.h"

int main() {
  // ATTENTION!!! : please use absolute path for reading the data file.
  const PointCloud pointcloud = ReadPointCloudFromTextFile(
      "/home/xmk/ponyai/PublicCourse/homework2/data/src.txt");
  WritePointcloudRangeToTextFile(pointcloud,
                                 "/home/xmk/ponyai/PublicCourse/homework2/data/src_range.txt");
  WritePointcloudHeightToTextFile(pointcloud,
                                  "/home/xmk/ponyai/PublicCourse/homework2/data/src_height.txt");
  std::cout << "Total points read: " << pointcloud.points.size() << std::endl;
  std::cout << "Rotation: " << std::endl;
  std::cout << pointcloud.rotation << std::endl;
  std::cout << "Translation: " << std::endl;
  std::cout << pointcloud.translation.transpose() << std::endl;
  return 0;
}

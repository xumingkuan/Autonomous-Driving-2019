// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <utility>
#include <vector>

#include "common/proto/map.pb.h"
#include "common/proto/route.pb.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "common/utils/math/vec2d.h"

#include "glog/logging.h"
#include "gflags/gflags.h"

DECLARE_string(map_dir);

namespace homework4 {
namespace map {

class MapLib {
 public:
  MapLib() {
    if (FLAGS_map_dir.empty()) {
      CHECK(file::ReadFileToProto("homework4/map/grid2/map_proto.txt", &map_data_));
    }  else {
      CHECK(file::ReadTextFileToProto(file::path::Join(FLAGS_map_dir, "map_proto.txt"), &map_data_));
    }
  }

  const interface::map::Map& map_proto() const { return map_data_; }
  std::unique_ptr<interface::map::Map> mutable_map_proto() {
    return std::make_unique<interface::map::Map>(map_data_);
  }

  interface::geometry::Point2D lane_point_2D(std::pair<int, int> id) const;
  std::pair<int, int> nearest_lane_point(const math::Vec2d& point) const;
  int initiate_successors();
  bool find_route_points(const std::pair<int, int>& start_lane_point,
                         const std::pair<int, int>& end_lane_point,
                         interface::route::Route& route);

 private:
  void add_route_points(interface::route::Route& route,
                        int lane_id, int start_id = 1, int end_id = -1) const;
  interface::map::Map map_data_;
  std::vector<std::vector<int> > successors_;
};

}  // namespace map
}  // namespace homework4

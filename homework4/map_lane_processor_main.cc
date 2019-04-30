// By xmk 2019.5.1

#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <utility>

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework4/map/map_lib.h"

DEFINE_string(map_dir, "/home/xmk/ponyai/PublicCourse/homework4/map/grid2/",
    "Directory path of map file");

class HashForPoint3D {
 public:
  std::size_t operator()(const interface::geometry::Point3D& point) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, point.x());
    boost::hash_combine(seed, point.y());
    boost::hash_combine(seed, point.z());
    return seed;
  }
};

class EqualToForPoint3D {
 public:
  bool operator()(const interface::geometry::Point3D& a,
      const interface::geometry::Point3D& b) const {
    return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
  }
};

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  std::unique_ptr<homework4::map::MapLib> map_lib = std::make_unique<homework4::map::MapLib>();
  std::unique_ptr<interface::map::Map> map_data = map_lib->mutable_map_proto();
  const int num_lane = map_data->lane_size();
  std::unordered_multimap<interface::geometry::Point3D, int, HashForPoint3D, EqualToForPoint3D>
      lane_start_point;
  for (int i = 0; i < num_lane; i++) {
    lane_start_point.insert(std::make_pair(map_data->lane(i).central_line().point(0), i));
  }
  int num_successor_pairs = 0;
  for (int i = 0; i < num_lane; i++) {
    auto next_lane =
        lane_start_point.equal_range(*map_data->lane(i).central_line().point().rbegin());
    for (auto it = next_lane.first; it != next_lane.second; it++) {
      map_data->mutable_lane(i)->add_successor()->set_id(map_data->lane(it->second).id().id());
      map_data->mutable_lane(it->second)->add_predecessor()->set_id(map_data->lane(i).id().id());
      num_successor_pairs++;
    }
  }
  CHECK(file::WriteProtoToTextFile(*map_data,
      file::path::Join(FLAGS_map_dir, "processed_map_proto.txt")));
  LOG(ERROR) << "Found " << num_successor_pairs << " pairs of successors in "
      << num_lane << " lanes.";
  return 0;
}
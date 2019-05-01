// By xmk 2019.5.1

#include <utility>

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework4/map/map_lib.h"

DEFINE_string(route_file_path,
    "/home/xmk/ponyai/PublicCourse/homework4/data/routes/route_request_5.txt",
    "Path of requested route");
DEFINE_string(route_result_path,
    "/home/xmk/ponyai/PublicCourse/homework4/data/routes/route_result_5.txt",
    "Path of result route");
DEFINE_string(map_dir, "/home/xmk/ponyai/PublicCourse/homework4/map/grid2/",
    "Directory path of map file");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_route_file_path.empty() || FLAGS_route_result_path.empty()) {
    LOG(ERROR) << "Empty route path.";
    return 0;
  }

  std::unique_ptr<homework4::map::MapLib> map_lib = std::make_unique<homework4::map::MapLib>();
  interface::route::Route route;
  CHECK(file::ReadTextFileToProto(FLAGS_route_file_path, &route)) << "Failed to load route file.";

  std::pair<int, int> start_lane_point =
      map_lib->nearest_lane_point(math::Vec2d(route.start_point()));
  std::pair<int, int> end_lane_point =
      map_lib->nearest_lane_point(math::Vec2d(route.end_point()));
  CHECK(map_lib->find_route_points(start_lane_point, end_lane_point, route))
      << "Failed to find a route.";
  CHECK(file::WriteProtoToTextFile(route, FLAGS_route_result_path))
      << "Failed to write result route file.";
  LOG(ERROR) << "Found a route with " << route.route_point_size() << " points.";
  return 0;
}
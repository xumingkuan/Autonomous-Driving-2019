// By xmk 2019.5.1

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework4/map/map_lib.h"

DEFINE_string(map_dir, "/home/xmk/ponyai/PublicCourse/homework4/map/grid2/",
    "Directory path of map file");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  std::unique_ptr<homework4::map::MapLib> map_lib = std::make_unique<homework4::map::MapLib>();
  int num_successor_pairs = map_lib->initiate_successors();
  CHECK(file::WriteProtoToTextFile(map_lib->map_proto(),
      file::path::Join(FLAGS_map_dir, "processed_map_proto.txt")));
  LOG(ERROR) << "Found " << num_successor_pairs << " pairs of successors in "
      << map_lib->map_proto().lane_size() << " lanes.";
  return 0;
}
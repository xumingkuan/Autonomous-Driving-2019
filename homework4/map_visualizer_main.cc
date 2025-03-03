// Copyright @2018 Pony AI Inc. All rights reserved.

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework4/display/main_window.h"
#include "homework4/map/map_lib.h"

DEFINE_string(route_file_path,
    "/home/xmk/ponyai/PublicCourse/homework4/data/routes/route_result_5.txt",
    "Path of displayed route");
DEFINE_string(map_dir, "/home/xmk/ponyai/PublicCourse/homework4/map/grid2/",
    "Directory path of map file");
// DEFINE_string(texture_dir, "/home/xmk/ponyai/PublicCourse/homework4/data/textures/",
//     "Directory path of texture file");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("pony.ai");
  QCoreApplication::setOrganizationDomain("pony.ai");
  QCoreApplication::setApplicationName("MapVisualizer");

  homework4::MainWindow main_window(nullptr);
  if (!FLAGS_route_file_path.empty()) {
    interface::route::Route route;
    CHECK(file::ReadTextFileToProto(FLAGS_route_file_path, &route)) << "Failed to load route file";
    main_window.set_displayed_route(route);
  }

  app.installEventFilter(&main_window);
  app.exec();

  return 0;
}

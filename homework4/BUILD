package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "map_lane_processor_main",
    srcs = ["map_lane_processor_main.cc"],
    deps = [
        "//common/utils/file",
        "//homework4/map:map_lib",
        "@gflags",
        "@glog",
    ],
)

cc_binary(
    name = "map_visualizer_main",
    srcs = ["map_visualizer_main.cc"],
    deps = [
        "//common/utils/file",
        "//homework4/display:main_window",
        "//homework4/map:map_lib",
        "@gflags",
        "@glog",
    ],
)

cc_binary(
    name = "route_finder_main",
    srcs = ["route_finder_main.cc"],
    deps = [
        "//common/utils/file",
        "//homework4/map:map_lib",
        "@gflags",
        "@glog",
    ],
)

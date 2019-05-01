package(default_visibility = ["//visibility:public"])

cc_library(
    name = "map_lib",
    srcs = ["map_lib.cc"],
    hdrs = ["map_lib.h"],
    deps = [
        "//common/proto:cc_map_proto",
        "//common/proto:cc_route_proto",
        "//common/utils/file",
        "//common/utils/math",
        "@boost//:functional",
        "@gflags",
        "@glog",
    ],
)

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "xumingkuan_agent",
    srcs = ["xumingkuan_agent.cc"],
    hdrs = ["xumingkuan_agent.h"],
    deps = [
        ":pid",
        "//common/proto:cc_agent_status_proto",
        "//common/proto:cc_control_proto",
        "//common/proto:cc_simulation_proto",
        "//homework5/simulation:vehicle_agent",
        "//homework5/simulation:vehicle_agent_factory",
        "@glog",
    ],
)

cc_library(
    name = "pid",
    hdrs = ["pid.h"],
)

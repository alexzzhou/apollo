package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "xsens",
    srcs = [
        "lib/libxsensdeviceapi.so",
        "lib/libxstypes.so",
    ],
    hdrs = [
        "include/xsensdeviceapi.h",
        "include/xstypes.h",
        "include/xscontroller.h",
        "include/xscommon.h",
    ],
    includes = [
        ".",
    ],
    include_prefix = "xsens",
)

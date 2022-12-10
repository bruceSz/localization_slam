cc_binary(
    name = "gauss_newton",
    srcs = ["gauss_newton.cc"],
    copts = [
        "-I/usr/include/eigen3 -I/usr/include/opencv4",
    ],
    linkopts = [
        "-lopencv_core ",
    ]

)

cc_binary(name = "ceres_basic",
    srcs = ["ceres_basic.cc"],
    copts = [
        " -I/usr/include/eigen3  -I/usr/include/opencv4",
    ],
     linkopts = [
        "-lopencv_core -lceres -lglog",
    ]


)

cc_binary(
    name = "g2o_b1",
    deps = [
        ":g2o_basic1",
    ]
)

cc_library(
    name = "g2o_basic1",
    srcs = ["g2o_basic1.cc"],
    hdrs = ["g2o_common.h"],
    copts = [
        "-std=c++14  -I/usr/include/eigen3 -I/usr/include/opencv4",
    ],
     linkopts = [
        "-L /usr/local/lib -lg2o_core -lg2o_stuff -lglog",
    ]

)

cc_binary(
    name = "g2o_b2",
    deps = [
        ":g2o_basic2"
    ],
)
cc_library(
    name = "g2o_basic2",
    srcs = ["g2o_basic2.cc"],
    hdrs = ["g2o_common.h"],
    copts = [
        "-std=c++14  -I/usr/include/eigen3 ",
    ],
     linkopts = [
        "-L /usr/local/lib -lg2o_core -lg2o_stuff -lglog",
    ]
)



cc_binary(
    name = "g2o_b3",
    srcs = ["g2o_basic3.cc"],
    copts = [
        "-std=c++14  -I/usr/include/eigen3 ",
    ],
     linkopts = [
        "-L /usr/local/lib -lg2o_core -lg2o_stuff -lglog",
    ]
)
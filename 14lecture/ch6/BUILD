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
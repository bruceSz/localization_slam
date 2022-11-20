#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>  // for formating strings
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>

using namespace std;
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 在pangolin中画图，已写好，无需调整
void showPointCloud(
    const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    TrajectoryType poses;         // 相机位姿

    if (argc != 3) {
        cout << "Usage: " << argv[0] << " pose.txt img_root" << endl;
        exit(1);
    }
    std::string pose_file = std::string(argv[1]);
    ifstream fin(pose_file);
    if (!fin) {
        cerr << "make sure pose file exists." << endl;
        return 1;
    }

    std::string data_root = std::string(argv[2]);
    cout << "root is: " << data_root << endl;

    for (int i = 0; i < 5; i++) {
        //boost::format fmt("./%s/%d.%s"); //图像文件格式
        boost::format fmt("%d.%s");
        boost::format fmt2("%d.%s");

        auto dep_file_name = (fmt % (i+1) % "pgm" );
        cout << "dep filename : " << dep_file_name << endl;

        auto img_file_name = (fmt2 % (i+1) % "png" );
        cout << "img_file_name is: " << img_file_name.str() << endl;
        
        //(fmt % "color" % (i + 1) % "png");
        auto img_file_path = data_root + "/color/" + img_file_name.str();
        auto dep_file_path = data_root + "/depth/" + dep_file_name.str();
        colorImgs.push_back(cv::imread(img_file_path));
        depthImgs.push_back(cv::imread(dep_file_path, -1)); // 使用-1读取原始图像

        double data[7] = {0};
        for (auto &d:data)
            fin >> d;
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                          Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(pose);
    }

    // 计算点云并拼接
    // 相机内参 
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);

    for (int i = 0; i < 5; i++) {
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cout << "color step is: " << color.step << endl;
        cv::Mat depth = depthImgs[i];
        Sophus::SE3d T = poses[i];
        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                if (d == 0) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;

                Vector6d p;
                p.head<3>() = pointWorld;
                p[5] = color.data[v * color.step + u * color.channels()];   // blue
                p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
                p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
                pointcloud.push_back(p);
            }
    }

    cout << "点云共有" << pointcloud.size() << "个点." << endl;
    showPointCloud(pointcloud);
    return 0;
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}

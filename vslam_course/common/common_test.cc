#include <fstream>
#include <string>
#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "camera.h"
#include "cam_mgr.h"
#include "pose.h"
#include "point.h"
#include "util.h"


void drawCube(const cv::Mat& img, const zs::Pose3D pose, std::string cam_conf, int cam_id)  {
    cv::Mat cube = loadCube(cam_conf);

    int r  = cube.rows, c = cube.cols;
    assert(r == 8 && c ==3);

    cv::Mat cubeImg(img.rows, img.cols, CV_8UC3);

    if(img.channels == 1) {
        std::vector<cv::Mat> channels(3, img);
        cv::merge(channels, cubeImg);
    } else {
        img.copyTo(cubeImg);
    }

    std::vector<cv::Point> pts;
    for(int i=0; i< r; i++) {
        double x = cube.at<double>(i, 0);
        double y = cube.at<double>(i, 1);
        double z = cube.at<double>(i, 2);

        zs::Point2D p = project(zs::Point3D(x, y, z), zs::CameraMgr::GetInstance().getCameraById(cam_id), pose);
        pts.push_back(zs::Point(p.x, p.y));

        cv::circle(cubeImg, cv::Point(p.x, p.y), 2, cv::Scalar(0,0,255), -1);

        line(cubeImg, points[0], points[1], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[1], points[2], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[2], points[3], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[3], points[4], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[4], points[5], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[5], points[6], cv::Scalar(0,0,255), 2);
        line(cudaImg, points[6], points[7], cv::Scalar(0,0,255), 2);
        line(cudaImg, points[7], points[4], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[0], points[4], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[1], points[5], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[2], points[6], cv::Scalar(0,0,255), 2);
        line(cubeImg, points[3], points[7], cv::Scalar(0,0,255), 2);
        
        return cubeImg;

    } 


}

void run(const cv::Mat & img, const zs::Pose3D pose, int cam_id) {
    int w = img.cols, h = img.rows;
    cv::Mat undist_img = zs::undist_image(img, zs::CameraMgr::GetInstance().getCameraById(cam_id));
    cv::Mat drawcube_img = drawCube(undist_img, pose);
    
    std::vector<cv::Mat> imgs{img, drawcube_img};

    cv::Mat merge_img = zs::mergeImage(imgs, w, h);

    cv::imshow("result", merge_img);
    cv::waitKey(25);
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cout << "Usage: draw_cube data conf"  << std::endl;
        return 0;
    }

    int cam_id = 0;
    std::string img_list_path = argv[1];
    std::string confPath = argv[2];
    std::string posePath = argv[3];


    zs::CameraPtr cam = std::shared_ptr<zs::Camera>(new zs::Camera(++cam_id, "default cam"));
    cam->loadC(confPath);



    std::vector<zs::Pose3D> poses;

    try {
        std::ifstream ifPose(posePath);
        if(!ifPose.is_open()) {
            std::cerr << "file not exist." << posePath << std::endl;
            return -1;
        }

        while(!ifPose.eof()) {
            double wx, wy, wz, tx, ty, tz;
            ifPose >> wx >> wy >> wz >> tx >> ty >> tz; 
            poses.push_back(zs::Pose3D(wx, wy, wz, tx, ty, tz));


        }

        ifPose.close();
    } catch(std::exception e) {
        std::cerr << "reading pose file failed:" << e.what() << std::endl;
        return -1;
    }

    try {
        std::ifstream img_list(img_list_path);
        if (!img_list.is_open()) {
            std::cerr << "image list file: " << img_list_path << " can not be opened." << std::endl;
            return -1;
        }
        int pose_idx = 0;

        while(!img_list.eof()) {
            std::string img_path;
            img_list >> img_path;
            cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);
            run(img, poses[pose_idx++]);
        }
        img_list.close();
    } catch (std::exception e) {
        std::cerr << "Failed to process img: " << e.what() << std::endl;
        return -1;
    }

    return 0;

}
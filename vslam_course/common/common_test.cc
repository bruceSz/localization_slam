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
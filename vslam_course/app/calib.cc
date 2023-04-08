#include <string>

#include "common/util.h"
#include "common/cam_mgr.h"
#include "common/draw_utils.h"
#include "math/dlt.h"

class Calib {
    public:
    Calib(const std::string& dataF, const std::string& conF): data_folder_(dataF), conf_folder_(conF) {}
    ~Calib() {}

    void init() {
        std::string cam_conf_path = conf_folder_ + "cam.yaml";
        cv::FileStorage fs(cam_conf_path, cv::FileStorage::READ);

        if(!fs.isOpened()) {
            std::cerr << "failed to load camera.yaml: " << cam_conf_path << std::endl;
            return;
        }

        float scale = 0.01f;

        cv::Mat corners;
        fs["corners"] >> corners;

        world_pts_.resize(corners.rows, corners.cols);

        for(int i=0; i<corners.rows; i++) {
            for(int j=0; j<corners.cols; j++) {
                world_pts_(i,j) = corners.at<double>(i,j) * scale;
            }
        }

        std::string project_px_path = data_folder_  + "detected_corners.txt";
        corner_pxs_ = zs::load_matrix(project_px_path, 120, 24);

        static zs::Camera::CameraPtr cam_ = std::shared_ptr<zs::Camera>(new zs::Camera(++camera_id, "default cam"));
        cam_->loadC(conf_folder_ + "camera.yaml");
        zs::CameraMgr::GetInstance().addCamera(cam_);

    }
    void run() {
        // read image
        // dlt solve M,
        //  case 1: intrinsic known.
        //  case 2: intrinsic unknown.
        // decompose M -> R, t
        // draw pose on image
        // draw reproject point on image

        
        try {
            std::string img_file = data_folder_ + "images.txt";
            std::ifstream fs(img_file.c_str());
            if(!fs.is_open()) {
                std::cerr << " not found " << img_file << std::endl;
                return;
            }

            int id =0 ;
            while(!fs.eof() && id < corner_pxs_.rows()) {
                std::string img_path;
                fs >> img_path ;

                img_path = data_folder_ + "images/" + img_path;
                cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);


                Eigen::MatrixXd px_line = corner_pxs_.row(id++);
                Eigen::MatrixXd pxs;

                // one row per image.
                pxs.resize(px_line.size() / 2, 2);

                for(int i=0; i< px_line.size()/2; i++) {
                    pxs(i,0) = px_line(2*i);
                    pxs(i,1) = px_line(2*i+1);
                }

                zs::Pose3D pose;
                auto cam = zs::CameraMgr::GetInstance().getCameraById(camera_id);
                Eigen::Matrix3d cam_k = cam->K();
                RunDLT(world_pts_, pxs, pose, cam_k);

                
                Eigen::Matrix<double, Eigen::Dynamic, 2> reproject_pxs;
                reproject_pxs.resize(world_pts_.rows(),2);

                for(int i=0; i< world_pts_.size(); i++) {
                    zs::Point3D pt(world_pts_(i,0), world_pts_(i,1), world_pts_(i, 2));
                    zs::Point2D px = zs::project(pt, cam, pose);
                    reproject_pxs(i,0)= px.x;
                    reproject_pxs(i,1) = px.y;
                }

                cv::Mat im1 = zs::drawPoint(img, pxs, zs::DrawType::CIRCLE, cv::Scalar(0,255,0));
                cv::Mat im2 = zs::drawPoint(im1, reproject_pxs, zs::DrawType::X, cv::Scalar(0,0,255));
                cv::imshow("result", im2);
                cv::waitKey(-1);

            }
            fs.close();
        } catch (std::exception& e) {
            std::cerr << "failed to process image:" << e.what() << std::endl;
        }

    }

    private:
    // do dlt to compute K.
    void RunDLT(const Eigen::Matrix<double, Eigen::Dynamic, 3> world_pts,
                const Eigen::Matrix<double, Eigen::Dynamic, 2> pxs,
                zs::Pose3D pose, 
                Eigen::Matrix3d & cam_K) {
        zs::DLT dlt(world_pts, pxs, cam_K );
        dlt.run();
        pose = dlt.get_pose();
        cam_K = dlt.get_cam_k();
    }

    std::string data_folder_, conf_folder_;
    int camera_id = 0;
    Eigen::MatrixXd world_pts_;
    Eigen::MatrixXd corner_pxs_;
};


int main(int argc, char** argv) {

    if(argc != 3) {
        std::cout << "Usage: calib data conf" << std::endl;
        return 0; 
    }


    std::string dataF = zs::folder_and_slash(argv[1]);
    std::string confF = zs::folder_and_slash(argv[2]);

    Calib calib(dataF, confF);
    calib.init();
    calib.run();
    return 0;

}
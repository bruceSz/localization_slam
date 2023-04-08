#include <iostream>
#include <vector>
#include "common/util.h"
#include "common/draw_utils.h"
#include "feature/harris.h"

//#include <eigen3/Eigen/SVD>
#include <Eigen/SVD>  
#include <Eigen/Dense>  

namespace zs {

class HarrisTrack {

public:
  HarrisTrack(const std::string& data_folder): data_folder_(data_folder) {}

  ~HarrisTrack() {}

  void run();

private:
  std::string data_folder_;

};

void HarrisTrack::run() {
    try {
        std::string img_path_list = data_folder_ + "images.txt";
        std::ifstream fs(img_path_list.c_str());

        if(!fs.is_open()) {
            std::cerr << "Couldn't open " << img_path_list << std::endl;
            return;
        }


        std::vector<cv::Point2i> last_frame_kps;
        std::vector<std::vector<uchar>> last_frame_descs;

        while(!fs.eof()) {
            std::string img_path;
            fs >> img_path;
            img_path = data_folder_ + "/" + img_path;
            cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

            zs::Harris harris;
            cv::Mat resp;
            std::vector<cv::Point2i> kps;
            std::vector<std::vector<uchar>> descs;
            std::vector<int> match_idx;
            harris.detect(img, resp, 3, 9, 0.08);
            harris.getKeyPoints(resp, kps, 200, 8); 
            harris.getDescriptors(img, kps, descs, 9);

            cv::Mat match_img;
            if (last_frame_descs.size() != 0 ) {
                harris.match(last_frame_descs, descs, match_idx, 5);
                match_img = harris.plotMatchOneImage(img, last_frame_kps, kps, match_idx);
            }

            last_frame_descs = descs;
            last_frame_kps = kps;

            cv::Mat kp_img = zs::drawPoint<cv::Point2i>(resp, kps, zs::DrawType::CIRCLE, cv::Scalar(0,255,255));
            std::vector<cv::Mat> merged_imgs;

            if(match_img.empty()) {
                merged_imgs.push_back(img);
                merged_imgs.push_back(kp_img);
            } else {
                merged_imgs.push_back(img);
                merged_imgs.push_back(match_img);
                merged_imgs.push_back(kp_img);
            }

            cv::Mat merge = zs::mergeImage(merged_imgs, img.cols/2, img.rows);
            cv::imshow("merge", merge);
            cv::waitKey(30);
        }
        fs.close();
    } catch(std::exception& e) {
        std::cerr << "image process exception: " << e.what() << std::endl;
    }
}

} // namespace zs

int example_jacobi() {  
    Eigen::Matrix3d A;  
    A(0,0)=1,A(0,1)=0,A(0,2)=1;  
    A(1,0)=0,A(1,1)=1,A(1,2)=1;  
    A(2,0)=0,A(2,1)=0,A(2,2)=0;  
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV );  
    Eigen::MatrixXd V = svd.matrixV(), U = svd.matrixU();  
    Eigen::MatrixXd  S = U.inverse() * A * V.transpose().inverse(); // S = U^-1 * A * VT * -1  
    std::cout<<"A :\n"<<A<<std::endl;  
    std::cout<<"U :\n"<<U<<std::endl;  
    std::cout<<"S :\n"<<S<<std::endl;  
    std::cout<<"V :\n"<<V<<std::endl;  
    std::cout<<"U * S * VT :\n"<<U * S * V.transpose()<<std::endl;  
    system("pause");  
    return 0;  
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "usage: " << argv[0] << std::endl;
        return 0;
    }
    //example_jacobi();


    std::string data_folder = zs::folder_and_slash(argv[1]);
    zs::HarrisTrack ht(data_folder);
    ht.run();
    return 0;
}
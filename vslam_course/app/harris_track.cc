#include <iostream>
#include <vector>
#include "common/util.h"
#include "feature/harris.h"

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

            cv::Mat kp_img = zs::drawPoint<cv::Point2i>(resp, kps, zs::DrawType::CIRCLE, cv::Scalar(0,0,255));
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
    } catch(std::exception e) {
        std::cerr << "image process exception: " << e.what() << std::endl;
    }
}

} // namespace zs

int main(int argc, char** argv) {
    int (argc != 2) {
        std::cout << "usage: " << argv[0] << std::endl;
        return 0;
    }

    std::string data_folder = zs::folder_and_slash(argv[1]);
    zs::HarrisTrack ht(data_folder);
    ht.run();
    return 0;
}
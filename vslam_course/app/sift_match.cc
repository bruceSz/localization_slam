#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>

#include "feature/sift.h"
#include "common/util.h"


namespace zs {

class SiftMatch {
  public:
    SiftMatch(const std::string data_folder): data_folder_(data_folder) {}
    ~SiftMatch() {}

    void run();

  private:
    std::string data_folder_;
};

void SiftMatch::run() {
    try {
        zs::Sift sift(200, 5, 3, 0.04, 10, 1.6);
        cv::Mat image = cv::imread(data_folder_ + "image1.jpg",cv::IMREAD_GRAYSCALE);
        cv::Mat image2 = cv::imread(data_folder_ + "image2.jpg",cv::IMREAD_GRAYSCALE);
        
        std::vector<cv::KeyPoint> kp_ref;
        std::vector<std::vector<float>> desc_ref;

        sift.detect(image, kp_ref, desc_ref);
        sift.plotGaussianPyramid("./output/gaussian_pyr_ref.png", true);
        sift.plotDogPyramid("./output/dog_pyr_ref.png", true);
        sift.plotKeypoints(image,  kp_ref,"./output/keypoints_ref.png", true);

        std::vector<cv::KeyPoint> kp_query;
        std::vector<std::vector<float>> desc_query;
        sift.detect(image2, kp_query, desc_query);
        sift.plotGaussianPyramid("./output/gaussian_pyr_query.png", true);
        sift.plotDogPyramid("./output/dog_pyr_query.png", true);
        sift.plotKeypoints(image2, kp_query ,"./output/keypoints_query.png", true);

        std::vector<int> match_idx;
        sift.match(desc_ref, desc_query, match_idx);
        sift.plotMatchTwoImage(image, image2, kp_ref, kp_query, match_idx, "./output/match_two_image.png", true);

    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
    }
    
}

}

int main(int argc, char **argv) {
    if (argc!= 2) {
        std::cout << "Usage: " << argv[0] << " data_folder" << std::endl;
        return 0;
    }

    std::string data_folder = zs::folder_and_slash( argv[1]);
    zs::SiftMatch match(data_folder);
    match.run();
    return 0;
}
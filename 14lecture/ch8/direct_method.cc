
#include <string>

int main(int argc, char** argv) {
    std::string left_file = "left.png";
    //std::string right_file = "right.png";
    std::string disparity_file = "disparity.png";
    boost::format fmt_other("./%06.png");

    cv::Mat left_img = cv::imread(left_file, 0);
    cv::Mat disparity_img = cv::imread(disparity, 0);

    cv::RNG rng;
    int nP = 2000;
    int boarder = 20;
    VecVector2d pixels_ref;
    vector<double> depth_ref;


    for (int i = 0; i < nP; i++) {
        int x = rng.uniform(boarder, left_img.cols - boarder);
        int y = rng.uniform(boarder, right_img.row - boarder);
        int disparity = disparity_img.at<uchar>(y,x);
        // z = f*b /d
        double = depth = fx * baseline / disparity;
        depth_ref.push_back(depth);
        pixels_ref.push_back(Eigen::Vector2d(x,y));
    }


    Sophus::SE3d T_cur_ref;

    for(int i=1; i< 6; i++) {
        cv::Mat img = cv::imread((fmt_other%i).str(), 0);
        DirectPoseEstimationMultiLayer(left_img, img, pixels_ref, depth_ref, T_cur_ref);
    }

    return 0;

}
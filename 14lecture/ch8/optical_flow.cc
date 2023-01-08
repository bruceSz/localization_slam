
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

using std::string;
using std::vector;

using cv::Mat;
using cv::imread;
using cv::KeyPoint;
using cv::GFTTDetector;

using cv::parallel_for_;
using cv::Range;

using cv::Point2f;


class OpticalFlowTracker {
    public:
    OpticalFlowTracker(const Mat& im1, 
                        const Mat& im2,
                        const vector<KeyPoint> & kp1,
                        vector<KeyPoint>& kp2,
                        vector<bool> & succ,
                        bool inverse = true, bool has_init = false) :
                        im1(im1), im2(im2), kp1(kp1), succ(succ), inverse(inverse),
                         has_init(has_init){}

    void calculateOpticalFlow(cosnt Range& range);
    private:
    const Mat& im1;
    const Mat& im2;
    const vector<KeyPoint> kp1;
    vector<KeyPoint> kp2;
    vector<bool> success;
    bool inverse = true;    
    bool has_init = false
};


void OpticalFlowTracker::calculateOpticalFlow(const Range& range) {


    int half_patch_size = 4;
    int iterations = 10;

    for(size_t i = range.start; i< range.end; i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0;
        if (has_init) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].py.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true;

        // hessian
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        // bias
        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        // jacobian
        Eigen::Vector2d J;



        for(int iter = 0; iter < iterations; iter++) {
            if (inverse == false) {
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            } else {
                b = Eigen::Vector2d::Zero();
            }

            cost = 0;

            for( int x = -half_patch_size; x < half_patch_size; x++) {
                for( int y = -half_patch_size; y < half_patch_size; y++) {
                    double error = GetPixelValue(im1, kp.pt.x+ x, kp.pt.y+ y) -
                                    GetPixelValue(im2, kp.pt.x + x + dx, kp.pt.y + y + dy);
                    if (inverse == false ) {
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(im2, kp.pt.x + dx + x +1, kp.pt.y + dy + y)-
                                    GetPixelValue(im2, kp.pt.x + dx + x -1, kp.pt.y + dy + y)),
                            0.5 * (GetPixelValue(im2, kp.pt.x + dx + x , kp.pt.y + dy + y +1)-
                                    GetPixelValue(im2, kp.pt.x + dx + x, kp.pt.y + dy + y -1))
                        );
                    } else if (iter ==0) {
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(im1, kp.pt.x + x +1,  kp.pt.y  + y) -
                                    GetPixelValue(im1, kp.pt.x + x -1, kp.pt.y + y)),
                            0.5 * (GetPixelValue(im1, kp.pt.x + x, kp.pt.y + y + 1 ) -
                                    GetPixelValue(im1, kp.pt.x + x, kp.pt.y + y -1))
                        );
                    }

                    b += -error * J;
                    cost  += error * error;
                    if (inverse == false || iter ==0) {
                        H += J * J.transpose();
                    }
                }
            }

            Eigen::Vector2d update = H.ldlt().solve(b);
             if (std::isnan(update[0])) {
                cout << "update is nan" << std::endl;
                succ = false;
                break;
             }

             if (iter > 0 && cost > lastCost) {
                break;
             }

             dx += update[0];
             dy += update[1];
             lastCost = cost;
             succ = true;
             if (update.norm() < 1e-2) {
                break;
             }

        }

        success[i] = succ;
        kp2[i].pt = kp.pt + Pointf(dx, dy);
    }
}

/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false,
    bool has_initial_guess = false
) {
    kp2.resize(kp1.size());
    success.resize(kp1.size());

    OpticalFlowTracker tracker(im1, im2, kp1, kp2, success, inverse, has_initial_guess);

    parallel_for_(Range(0,kp1.size()),
                    std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, 
                    std::placeholders::_1));
}


void OpticalFlowMultiLevel(
    const Mat &im1,
    const Mat &im2,
    const vector<KeyPoint> & kp1,
    vector<KeyPoint> & kp2,
    vector<bool>  &success,
    bool inverse ) {

        int pyramids = 4;
        double pyramid_scale = 0.5;
        double scales[] = {1.0, 0.5, 0.25, 0.125};

        // image pyramids
        vector<Mat> pyr1, pyr2;

        for(int i=0; i< pyramids; i++) {
            if (i == 0) {
                pyr1.push_back(im1);
                pyr2.push_back(im2);
            } else {
                Mat im1_pyr, im2_pyr;
                cv::resize(pyr1[i-1], im1_pyr, 
                            cv::Size(pyr1[i-1].cols * pyramid_scale, pyr1[i-1].rows*pyramid_scale));
                cv::resize(pyr2[i-1], im2_pyr,
                            cv::Size(pyr2[i-1].cols * pyramid_scale, pyr2[i-1].rows* pyramid_scale));

                pyr1.push_back(im1_pyr);
                pyr2.push_back(im2_pyr);
            }

        }

        // smallest kp in pyramid.
        vector<KeyPoint> kp1_pyr, kp2_pyr;
        for(auto & kp: kp1) {
            auto kp_top = kp;
            kp_top.pt *= scales[pyramids - 1];
            kp1_pyr.push_back(kp_top);
            kp1_pyr.push_back(kp_top);
        }

        for(int level = pyramids -1; level >= 0; level--) {
            //from coarse to fine

            success.clear();
            OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);

            if (level > 0) {
                for(auto &kp: kp1_pyr) {
                    kp.pt /= pyramid_scale;
                }
                for(auto &kp: kp2_pyr) {
                    kp.pt /= pyramid_scale;
                }
            }

        }

        for(auto & kp: kp2_pyr) {
            kp2.push_back(kp);
        }


}

int main(int argc, char** argv) {
    string file1 = "./LK1.png";
    string file2 = "./LK2.png";

    Mat im1 = imread(file1, 0);
    Mat im2 = imread(file2, 0);
    cv::Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20);
    detector->detect(im1, kp1);


    vector<KeyPoint> kp2_single;
    vector<bool> succ_single;

    OpticalFlowSingleLevel(im1, im2, kp1, kp2_single, succ_single);

    vector<KeyPoint> kp2_multi;
    vector<bool> succ_multi;

    OpticalFlowMultiLevel(im1, im2, kp1, kp2_multi, succ_multi, true);
    cout << "optical flow by gauss-newton" << endl;


    vector<Point2f> pt1, pt2;
    for( auto kp: kp1) {
        pt1.push_back(kp.pt);
    }

    vector<uchar> statue;
    vector<float> error;

    cv::calcOpticalFlowPyrLK(im1, im2, kp1, pt2, status, error);
    cout << "optical flow by opencv" << endl;



    Mat im2_single;
    cv::cvtColor(im2, img2_single, CV_GRAY2BGR);
    for(int i=0; i< kp2_single.size(); i++) {
        if(succ_single[i]) {
            cv::circle(im2_single, kp2_single[i].pt, 2, cv::Scalar(0,250,0),2);
            cv::line(im2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0,250,0));
        }
    }



    Mat im2_multi;
    cv::cvtColor(im2, im2_multi, CV_GRAY2BGR);
    for(int i=0; i< kp2_multi.size(); i++) {
        if (succ_multi[i]) {
            cv::circle(im2_multi, kp2_multi[i].pt, 2, cv::Scalar(0,250,0),2);
            cv::line(im2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0,250,0));
        }
    }


    Mat im2_cv;
    cv::cvtColor(im2, im2_cv, CV_GRAY2BGR);
    for(int i=0;i< pt2.size(); i++) {
        if (status[i]) {
            cv::circle(im2_cv, pt2[i].pt, 2, cv::Scalar(0,250,0),2);
            cv::line(im2_cv, pt1[i], pt2[i], cv::Scalar(0,250,0));
        }
    }

    cv::imshow("tracked single level", im2_single);
    cv::imshow("tracked multi level", im2_multi);
    cv::imshow("tracked opencv", im2_cv);
    cv::waitKey(0);
    return 0;

    
}
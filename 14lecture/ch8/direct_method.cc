
#include <string>

class JacobianAccumulator {

public:
    JacobianAccumulator(
        const cv::Mat& im1_, 
        const cv::Mat& im2_,
        const VecVector2d& px_ref_,
        const vector<double> depth_ref_,
        Sophus::SE3d& T21_ ):
        im1(im1_), im2(im2_), px_ref(px_ref_),
        depth_ref(depth_ref_),T21(T21_) {
            projection_ = VecVector2d(px_ref.size(), Eigen::Vector2d(0,0));
    }

    void accumulate_jacobian(const cv::Range & range) ;
    
    Matrix6d hessian() const { return H;}

    Vector6d bias() const { return b;}

    double cost_func() const { return cost;}

    VecVector2d projected_points() const { return projection_;}

    void reset() {
        H = Matrix6d::Zero();
        b = Vector6d::Zero();
        cost = 0;
    }

private:
    const cv::Mat & im1;
    const cv::Mat & im2;
    const VecVector2d & px_ref;
    const vector<double> depth_ref_;
    Sophus::SE3d &T21;

    VecVector2d projection_;

    std::mutex hessian_mutex;
    Matrix6d H = Matrix6d::Zero();
    Vector6d b = Vector6d::Zero();
    double cost = 0;
};

void DirectPoseEstimationSingleLayer(
    const cv::mat & im1,
    const cv::Mat & im2,
    const VecVector3d& px_ref,
    const vector<double> depth_ref,
    Sophus::SE3d &T21 {
        const int iterations = 10;
        double const = 0, lastCost = 0;
        auto t1 = chrono::steady_clock::now();
        JacobianAccumulator jaco_accu(im1, im2, px_ref, depth_ref, T21);


        for( int iter = 0; iter < iterations; iter++) {
            jaco_accu.reset();
            cv::parallel_for_(
                cv::Range(0,px_ref.size()),
                std::bind(&JacobianAccumulator::accumulate_jacobian, &jaco_accu, std::placeholders::_1)
            );

            Matrix3d H = jaco_accu.hessian();
            Vector6d b = jaco_accu.bias();

            Vector6d update = H.ldlt().solve(b);

            T21 = Sophus::SE3d::exp(update)*T21;
            cost  = jaco_accu.cost_func();

            if (std::isnan(update[0])) {
                cout << "update is nan" << std::endl;
                break;
            }

            if (iter > 0 && cost > lastCost) {
                cout << "cost increased: " << cost << ", " << lastCost << std::endl;
                break;
            }

            if (update.norm() < 1e-3) {
                // update is too small, 
                // local minimum.
                break;
            }

            lastCost = cost;
            cout << "iter: " << iter << ", cost: " << cost  << endl;

        }

        cout << "T21 = \n " << T21.matrix() << std::endl;
        cv::Mat img2_show;
        cv::cvtColor(im2, img2_show, CV_GRAY2BGR);
        VecVector2d projection = jaco_accu.projected_points();

        for(size_t i=0; i< px_ref.size(); i++) {
            auto p_ref = px_ref[i];
            auto p_cur = projection[i];
            if (p_cur[0]>0 && p_cur[1]> 0 ) {
                cv::circl(img2_show, cv::Point2f(p_cur[0], p_cur[1]), 2, cv::Scalar(0,250,0),2);
                cv::line(img2_show, cv::Point2f(p_ref[[0], p_ref[1]), cv::Point2f(p_cur[0], p_cur[1]), 
                    cv::Scalar(0,250,0));
            }
        }

        cv::imshow("current", img2_show);
        cv::waitKey();x
}
)

void DirectPoseEstimationMultiLayer(
    const cv::Mat & im1,
    const cv::Mat & im2,
    const VecVector2d & px_ref,
    const vector<double> double_ref,
    Sophus::SE3D & T21)  {

    int pyramids = 4;

    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    vector<cv::Mat> pyr1, pyr2;

    for(int i=0; i< pyramids; i++) {
        if (i==0) {
            pyr1.push_back(im1);
            pyr2.push_back(im2);
        } else {
            cv::Mat im1_pyr, im2_pyr;
            cv::resize(pyr1[i-1], img1_pyr, 
                        cv::Size(pyr1[i-1].cols * pyramid_scale, pyr1[i-1].rows* pyramid_scale));
            cv::resize(pyr2[i-1], img2_pyr, 
                        cv::Size(pyr2[i-1].cols * pyramid_scale, pyr2[i-1].rows* pyramid_scale));

            pyr1.push_back(im1_pyr);
            pyr2.push_back(im2_pyr);
            
        }
    }

    // backup.
    double fxG = fx, fyG = fy, cxG = cx, cyG = cy;

    for(int level = pyramids-1; level >= 0; level--) {

        VecVector2d px_ref_pyr;
        for(auto & px: px_ref) {
            px_ref_pyr.push_back(scales[level]* px);
        }

        fx = fxG * scales[level];
        fy = fyG * scales[level];
        cx = cxG * scales[level];
        cy = cyG * scales[level];

        DirectPoseEstimationSingleLayer(pyr1[level], pyr2[level], px_ref_pyr, depth_ref, T21);
    }


}

void JacobianAccumulator::accumulate_jacobian(const cv::Range& range) {
    const int half_patch_size = 1;
    int cnt_good = 0;
    Eigen::Matrix6d hessian = Vector6d::Zero();
    Eigen::Vector6d bias = Vector6d::Zero();

    double cost_tmp = 0;

    for(size_t i=range.start; i< range.end; i++) {
        Eigen::Vector3d p_ref = 
            depth_ref[i] * Eigen::Vector3d((px_ref[i][0]-cx)/fx, (px_ref[i][1]-cy)/fy, 1);
        Eigen::Vector3d point_cur = T21 *  p_ref;
        if (point_cur[2] < 0) {
            // depth invalid.
            continue;
        }

        float u = fx * point_cur[0] / point_cur[2]+ cx;
        float v = fy * point_cur[1] / point_cur[2] + cy;
        if(u < half_patch_size || u > img2.cols - half_patch_size ||
            v < half_patch_size || u > img2.rows - half_patch_size) {
                continue;
            }
        projection[i] = Eigen::Vector2d(u,v);

        double X = point_cur[0], Y = point_cur[1], Z = point_cur[2];
        Z2 = Z*Z, Z_inv = 1.0/Z, Z2_inv = Z_inv * Z_inv;

        cnt_good++;

        for(int x = -half_patch_size;x <= half_patch_size;x++) {
            for(int y=-half_patch_size;y <= half_patch_size;y++) {
                double error = GetPixelValue(img1, px_ref[i][0]+x, px_ref[i][1]+y ) - 
                                GetPixelValue(img2, u + x, v + y);
                Eigen::Matrix26d J_pixel_xi;
                Eigen::Vector2d J_img_pixel;

                J_pixel_xi(0,0) = fx * Z_inv;
                J_pixel_xi(0,1) = 0;
                J_pixel_xi(0,2) = -fx * X  Z2_inv;
                J_pixel_xi(0,3) = - fx * X * Y * Z2_inv;
                J_pixel_xi(0,4) = fx  + fx * X * X * Z2_inv;
                J_pixel_xi(0,5) = - fx * * Y * Z_inv;

                J_pixel_xi(1,0) = 0
                J_pixel_xi(1,1) = fy * Z_inv;
                J_pixel_xi(1,2) = -fy * Y  Z2_inv;
                J_pixel_xi(1,3) = -fy  - fy * Y * Y * Z2_inv;
                J_pixel_xi(1,4) = fy * X * Y * Z2_inv;
                J_pixel_xi(1,5) =  fy * X  * Z_inv;


                J_img_pixel = Eigen::Vector2d(
                    0.5 * (GetPixelValue(img2, u + 1 + x, v + y)- GetPixelValue(img2, u - 1 + x, v + y)),
                    0.5 * (GetPixelValue(img2, u + x, v + y + 1) - GetPixelValue(img2, u + x, v + y - 1)),
                );

                // this is column vector. 6 line.
                Vector6d J = -1.0 * (J_img_pixel.transpose() * J_pixel_xi).transpose();
                hessian += J * J.transpose();
                bias += -error * J;
                cost_tmp += error * error;

            }
        }
        if (cnt_good) {
            unique_lock<mutex> l(hessian_mutex);
            H += hessian;
            b += bias;
            cost += cost_tmp/cnt_good;
        }
    }

}

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
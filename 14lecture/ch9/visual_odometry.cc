#include "visual_odometry.h"


namespace slam_fe {
VisualOdometry::VisualOdometry( std::string settingPath, Viewer* viewer ):
    state_(INIT), ref_(nullptr), curr_(nullptr),map_(nullptr),
    num_lost_(0), num_inliers_(0), mpViewer(viewer) {
        cv::FileStorage setting(settingPath, cv::FileStorage::READ);
        
        num_of_fts = setting["num_of_fts"];
        scale_factor = setting["scale_factor"];
        level_pyramid = setting["level_pyramid"];
        match_ratio = setting["match_ratio"];
        max_num_lost   = setting["max_num_lost"];
        min_inliers = setting["min_inliers"];
        key_frame_min_rotation = setting["key_frame_min_rotation"];
        key_frame_min_trans = setting["key_frame_min_trans"];
        setting.release();

        mpCamera = new slam_fe::Camera(settingPath);

        orb_ = cv::ORB::create(num_of_fts, scale_factor, level_pyramid);
        frameid = 0;

}

VisualOdometry::~VisualOdometry() {}



cv::Mat VisualOdometry::Tracking(cv::Mat im, cv::Mat imD, double tFrame) {
    Frame* pFrame = new Frame(frameid);
    pFrame->color_ = im;
    pFrame->depth_ = imD;
    pFrame->camera_ = mp_camera;
    pFrame->time_stamp_ = tFrame;

    cv::Mat T_c_w = addFrame(pFrame);
    frameid++;

    return T_c_w;

}


cv::Mat VisualOdometry::addFrame(Frame* frame ) {
    cv::Mat T_c_w;

    if (state_ == INIT) {
        std::unique_ptr<std::mutex> lock(mpView->mMutexViewer);
        curr_ = ref_ = frame;

        ref_->T_c_w_ = cv::Mat::eye(4,4, CV_32FC1);
        curr_->T_c_w_ = cv::Mat::eye(4,4, CV_32FC1);

        map_->insertKeyFrame(frame);

        ExtractORB();

        setRef3DPoints();

        state_ = OK;

        return curr_->T_c_w_;

    } else {

        if (state_ == LOST) {
            std::cout << "vo has lost." << std::endl;
            num_lost++;
            if (num_lost_ > max_num_lost_) {
                state_ = LOST;
            }
            return ref_->T_c_w_;
        } else {
            curr_ = frame;
            ExtractORB();

            featureMatching();

            poseEstimationPnp();

            if (checkEstimatedPose() == true) {
                curr_->T_c_w = T_c_r_estimate * ref_->T_c_w_;
                ref_ = curr_;

                setRef3DPoints();
                num_lost_ = 0;

                if (checkKeyFrame() == true) {
                    addKeyFrame();
                }
            }


            mvAllFrame.push_back(curr_);

            cv::Mat T_c_w = curr_->T_c_w_;

            mpViewer->SetCurrentCameraPose(T_c_w);
            mpViewer->GetAllFrame(mvAllFrame);
            mpViewer->GetAll3dPoints(pts_3d_all);
            mpViewer->SetVisualOdometry(this);

            cv::waitKey(10);

            return T_c_w;

        }

    }

}


void VisualOdometry::ExtractORB() {
    orb_->detectAndCompute(curr_->color_, cv::Mat(), key_points_curr_, desc_curr_);
}

void VisualOdometry::featureMatching() {
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    
    matcher.match(desc_ref_, desc_curr_, matches);

    float max_dis = 0;
    for(int i=0; i< matches.size(); i++) {
        if (matches[i].distance > max_dis) {
            max_dis = matches[i].distance;
        }
    }

    feature_matches_.clear();
    for(cv::DMatch& m: matches) {
        if (m.distance < max_dis * 0.3) {
            feature_matches_.push_back(m);
        }
    }

    std::cout << "good matches: " << feature_matches_.size()  << std::endl;

}


void VisualOdometry::setRef3DPoints() {
    pts_3d_ref.clear();
    desc_ref_ = cv::Mat();
    for(int i=0 ;i < key_points_curr_.size(); i++) {
        double d = ref_->findDepth(key_points_curr_[i]);
        cv::Point3f p_cam = ref_->camera->pixel2camera(cv::Point2f(key_points_curr_[i].pt.x, key_points_curr_[i].pt.y),d);
        pts_3d_ref.push_back(p_cam);
        pts_3d_all.push_back(p_cam);
        desc_ref_.push_back(desc_curr_.row(i));

    }

}

void VisualOdometry::poseEstimationPnp() {
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;

    for(int i=0; i< feature_matches_.size() ; i++) {
        // not in current scope.
        if (pts_3d_ref[feature_matches_[i].queryIdx].z < 0) {
            continue;
        }

        pts3d.push_back(pts_3d_ref[feature_matches_[i].queryIdx]);
        pts2d.push_back(key_points_curr_[feature_matches_[i].trainIdx].pt);

    }



    cv::Mat K = (cv::Mat<float>(3, 3) << ref_->camera->fx_, 0.0, ref_->camera->cx_,
                0.0, ref_->camera->fy_, ref->camera->cy_,
                0.0, 0.0, 1.0);
    
    cv::Mat rvec = cv::Mat::zeros(3,1,CV_32FC1);
    cv::Mat tvec = cv::Mat::zeros(3,1, CV_32FC1);

    std::vector<int> inliers;


    cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);

    num_inliers_ = inliers.size();

    std::cout << "pnp inliers: " << inliers.size() << std::endl;

    cv::Rodrigues(rvec, rvec);

    T_c_r_estimate = cv::Mat::eye(4, 4, CV_32FC1);

    rvec.copyTo(T_c_r_estimate(cv::Rect(0,0, 3, 3)));
    tvec.copyTo(T_c_r_estimate(cv::Rect(3,0, 1, 3)));

}


void VisualOdometry::addKeyFrame() {
    std::cout << "adding a key-frame" << std::endl;
    map_->insertKeyFrame(curr_);
}


bool VisualOdometry::checkEstimationPose() {
    if (num_inliers_ < min_inliers) {
        std::cout << "reject because inlier is too small: " << num_inliers_ << std::endl;
        return false;
    }

    cv::Mat rvec = T_c_r_estimate(cv::Rect(0,0,3,3));
    cv::Mat tvec = T_c_r_estimate(cv::Rect(3,0,1,3));

    // here determinant f rvec should be 1.
    // if it deviates too much , rvec is not rotation matrix anymore.
    if (tvec.at<double>(0,0) > 20.0 || abs(1.0-determinant(rvec)) > 0.01) {
        std::cout << "reject because motion is too large." << std::endl;
        return false;
    }
    return true;
}


bool VisualOdometry::checkKeyFrame() {
    cv::Mat rvec = T_c_r_estimate(cv::Rect(0,0,3,3));
    cv::Mat tvec = T_c_r_estimate(cv::Rect(3,0, 1,3));

    cv::Scalar t = cv::trace(rvec);
    double trR = t.val[0];
    double theta = acos((trR-1.0)/2.0);


    if (abs(theta) > key_frame_min_rot || norm(tvec) > key_frame_min_trans) {
        return true;
    }

    return false;
}

} // namespace slam_fe
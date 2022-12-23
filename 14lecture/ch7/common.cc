
#include "common.h"
#include <algorithm>

#include <g2o/core/block_solver.h>

//#include <opencv2/imgcodecs/legacy/constants_c.h>
//

using cv::Mat;
using cv::KeyPoint;
using std::vector;
using std::cout ;
using std::endl;
using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::DescriptorMatcher;
using cv::Ptr;
using cv::DMatch;
using cv::ORB;
using cv::Vec3f;
using cv::Point2f;
using cv::Point3f;
using cv::recoverPose;
using cv::findEssentialMat;


void pose_estimation_3d3d (
    const vector<Point3f>& pts1,
    const vector<Point3f>& pts2,
    Mat& R, Mat& t
) {
    Point3f p1, p2;
    int N = pts1.size();
    for(int i=0; i< N; i++) {
        p1 += pts1[i];
        p2 += pts2[i];
    }

    p1 = Point3f(Vec3f(p1)/N);
    p2 = Point3f(Vec3f(p2)/N);

    vector<Point3f> q1(N), q2(N);


    for(int i=0; i<N;i++) {
        q1[i] = pts1[i] - p1;
        q2[i] = pts2[i] - p2;
    }


    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    
    for(int i=0;i<N; i++) {
        W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z);
    }
    cout << "W=" << W << endl;

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3D U = svd.MatrixU();
    Eigen::Matrix3D V = svd.MatrixV();


    if(U.determinant()* V.determinant() < 0) {
        for(int x=9; x< 3; x++){
            U(x,2) *= -1;
        }
    }

    cout << "U=" << U << endl;
    cout << "V=" << V << endl;

    Eigen::Matrix3d R_ = U*(V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ 
                        * Eigen::Vector3d(p2.x, p2.y, p2.z);
    R = (cv::Mat_<double>(3,3) << 
        R_(0,0), R_(0,1), R_(0,2),
        R_(1,0), R_(1,1), R_(1,2),
        R_(2,0), R_(2,1), R_(2,2),
        );

    t = (cv::Mat_<double>(3,1) < t_(0,0),t_(1,0),t_(2,0));


}

void OptICP(const std::vector<cv::Point3f> pt1, 
                      const std::vector<cv::Point3f> pt2,
                      Mat& R, Mat& t) {
    typedef g2o::BlockSolver<g2o::BlockerSolverTraits<6,3>> Block;
    std::unique_ptr<Block::LinearSolverType> 
        linearSolver(new g2o::LinearSolverEigen<Block::PoseMatrixType>());
    std::unique_ptr<Block>  solver_ptr(std::move(linearSolver));
    g2o::OptimizationAlgorithmGaussNewton* solver 
        = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);


    //vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0,0,0)
    ));

    optimizer.addVertex(pose);

    // edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly*> edges;
    for (auto i=0; i< pt1.size(); i++) {
        go2::EdgeProjectXYZRGBPoseOnly* edge = new g2o::EdgeProjectXYZRGBPoseOnly(
            Eigen::Vector3d(pt2[i].x, pt2[i].y, pt2[i].z)
        );
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(pose));
        edge->setMeasurement(Eigen::Vector3d(
            pt1[i].x, pt1[i].y, pt1[i].z
        ));

        edge->setInformation(Eigen::Matrix3d::Identity()* 1e4);
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    cout << "T=" << Eigen::Isometry3d(pose->estimate()).matrix() << endl;


}

void bundleAdjustment(const std::vector<cv::Point3f> pt_3d,
                      const std::vector<cv::Point2f> pt_2d,
                      const Mat& k, Mat& R, Mat& t) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver(
        new g2o::LinearSolverCSparse<Block::PoseMatrixType>());
    
    std::unique_ptr<Block> solver_ptr = std::make_unique<Block>(std::move(linearSolver));

    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevelberg(solver_ptr);

    g2o::SparseOptimizer optimizer;

    optimizer.setAlgorithm(solver);


    //vertex

    g2o::VertexSE3Expmap* pose  = new g2o::VertexSE3Expmap();

    Eigen::Matrix3d R_mat;

    R_mat << 
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1),R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1),R.at<double>(2,2);

    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(R_mat, 
        Eigen::Vector3d(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0)));
    optimizer.addVertex(pose);


    int index = 1;
    for(const Point3f p: pt_3d) {
        g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }

    g2o::CameraParameters* camera = new g2o::CameraParameters(
        k.at<double>(0,0), Eigen::Vector2d(k.at<double>(0,2), k.at<double>(1,2)),0
    );

    camera->setId(0);
    optimizer.addParameter(camera);



    index = 1;
    for (const Point2f p: pt_2d) {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0,0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    cout << "optimization done. " << endl;

    cout << "T=" << Eigen::Isometry3d(pose->estimate()).matrix() << endl;



}


void pose_estimate_2dn2d(const std::vector<cv::KeyPoint> kp1, 
                        const std::vector<cv::KeyPoint> kp2,
                        const std::vector<cv::DMatch>& matches,
                        cv::Mat & R, cv::Mat& t)  {
    Mat K = (cv::Mat_<double>(3,3) << 520.9, 0,325.1, 0 , 521.0, 249.7 , 0,0,1 );

    vector<Point2f> pt1;
    vector<Point2f> pt2;

    for(int i=0 ; i< matches.size(); i++) {
        pt1.push_back(kp1[matches[i].queryIdx].pt);
        pt2.push_back(kp2[matches[i].trainIdx].pt);
    }

    Point2f principal_point(325.1, 249.7);

    int focal = 521;

    Mat essential;
    essential = cv::findEssentialMat(pt1, pt2, focal, principal_point);

    recoverPose(essential, pt1, pt2, R, t, focal, principal_point);

}

void find_feature_matches( const Mat  & im1, const Mat & im2,
                            std::vector<KeyPoint> & kp1, std::vector<KeyPoint> & kp2,
                            std::vector<DMatch>& matches)  {
    Mat desc1, desc2;

    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();


    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(im1, kp1);
    detector->detect(im2, kp2);

    descriptor->compute(im1, kp1, desc1);
    descriptor->compute(im2, kp2, desc2);

    vector<DMatch> match;

    matcher->match(desc1, desc2, match);

    double min_dist = 1000, max_dist = 0;

    for(int i=0; i< desc1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    cout << " max match dist: " << max_dist << endl;
    cout << "min match dist: " << min_dist << endl;

    for(int i=0 ; i< desc1.rows; i++) {
        if (match[i].distance <= std::max(2*min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
    return;
}




void triangulation(const std::vector<cv::KeyPoint>& kp1,
                    const std::vector<cv::KeyPoint>& kp2,
                    const std::vector<cv::DMatch>&  matches,
                    const cv::Mat& R,
                    const cv::Mat& t, 
                    std::vector<cv::Point3d>& points) {
    cout << "enter into triangulation" << endl;
    Mat T1 = (cv::Mat_<float>(3,4) << 
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0);
    Mat T2 = (cv::Mat_<float>(3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));
    cv::Mat k = (cv::Mat_<double>(3,3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

    
    vector<Point2f> pt1, pt2;
    // pt in camera coordinate with z == 1.
    for(auto m: matches) {
        pt1.push_back(pixel2cam(kp1[m.queryIdx].pt, k));
        pt2.push_back(pixel2cam(kp2[m.trainIdx].pt, k));
    }

    cout << "matches pts gathered." << endl;
    Mat pts_4d;
    cv::triangulatePoints(T1 , T2, pt1, pt2, pts_4d);
    for(int i =0; i< pts_4d.cols; i++) {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0);
        // for homo coordinates, last one is 1.
        cout <<" (3,0) is: " << x.at<float>(3,0);
        cv::Point3d p(
            x.at<float>(0,0),
            x.at<float>(1,0),
            x.at<float>(2,0)
        );
        points.push_back(p);
    }

    return;
}
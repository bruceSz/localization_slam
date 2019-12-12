//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <iomanip>  

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>

#include <gflags/gflags.h>

DEFINE_int32(n_frame, 2, "number of key frame.");
DEFINE_double(noise_w, 0.0, "weight of measure noise, used to control wether use noise or not.");
DEFINE_double(noise_var, 2.0, "variance of measure noise");

using namespace cv;

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标

};


void multiFrameTriangulateP(std::vector<Pose>& pose_v, std::vector<Eigen::Vector2d>& image_p_v, Eigen::Vector3d& point_3d, int n_frame) {
    assert(n_frame < pose_v.size());
    assert(pose_v.size() == image_p_v.size());
    Eigen::MatrixX4d design_matrix ;
    design_matrix.resize(n_frame*2, Eigen::NoChange);
    for(int i=0; i < n_frame ; ) {
        // rwc.transpose() should be from rotation from world to camera?
        // -twc should be translation from world to camera?
        //  it turn out that if there is no rotation at all rcw = I , twc = - tcw
        //  while if there is a rotation, we need to first do rotation on the twc then adding a minus sign to it.
        auto R  = pose_v[i].Rwc.transpose();
        auto t  =  - R * pose_v[i].twc;
        Eigen::Matrix<double, 3,4> Pose;
        Pose <<
            R(0,0), R(0,1), R(0,2), t(0,0),
            R(1,0), R(1,1), R(1,2), t(1,0),
            R(2,0), R(2,1), R(2,2), t(2,0);
            // D(i*2) = ui * pose(2) - pose(0)
            // D(i*2+1) = vi * pose(2) - pose(1)
        design_matrix.row(i*2) = image_p_v[i][0] * Pose.row(2) - Pose.row(0);
        design_matrix.row(i*2+1) = image_p_v[i][1] * Pose.row(2) - Pose.row(1);
        i++;
    }

    Eigen::Vector4d triangulated_point;
    auto res = design_matrix.jacobiSvd(Eigen::ComputeFullV);
    triangulated_point =
              res.matrixV().rightCols<1>();

    std::cout<< "Singular values: \n " << res.singularValues()  << std::endl;
    std::cout << "ratio of smallest singular value and second smallest singular value : "
            << std::setprecision(20) << res.singularValues()(3) / res.singularValues()(2)
             << std::endl;

    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);

}

void triangulatePoint(const Pose& pose0_r, const Pose& pose1_r,
    /*Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,*/
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    //1. compute D matrix , using projection relationship, each camera pose relate to two line.
    auto R = pose0_r.Rwc;
    auto t = pose0_r.twc;
    //Mat R;
    //Mat t;
    //std::cout << "before conv: " << eiR ;
    //cv::eigen2cv(eiR, R);
    //cv::eigen2cv(eit, t);
    Eigen::Matrix<double, 3,4> Pose0;
    Pose0 <<
        R(0,0), R(0,1), R(0,2), t(0,0),
        R(1,0), R(1,1), R(1,2), t(1,0),
        R(2,0), R(2,1), R(2,2), t(2,0);

    //Eigen::Matrix<double, 3,4> Pose0;
    //cv::cv2eigen(eiPose0, Pose0);
    
    //eiR = pose1_r.Rwc;
    //eit= pose1_r.twc;
    R = pose1_r.Rwc;
    t= pose1_r.twc;
    //cv::eigen2cv(eiR, R);
    //cv::eigen2cv(eit, t);

    Eigen::Matrix<double, 3,4> Pose1 ;
    Pose1 <<
        R(0,0), R(0,1), R(0,2), t(0,0),
        R(1,0), R(1,1), R(1,2), t(1,0),
        R(2,0), R(2,1), R(2,2), t(2,0);

    //Eigen::Matrix<double, 3,4> Pose1;
    //cv::cv2eigen(eiPose1, Pose1);
    //2. svd the D matrix, output singular values , find the smallest. singlar value. 
    //  using jacobiSvd

    //3. validate the result: comparing u3 and u4 
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


int main(int argc, char** argv)
{

    google::ParseCommandLineFlags(&argc,&argv, false);
    int poseNums = 100;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);

    std::normal_distribution<double> noise_normal(0.0, FLAGS_noise_var);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    
    std::vector<Pose> camera_pose_ft;
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        // add some noise to the pw
        auto noise = noise_normal(generator);
        //std::cout << "Adding noise : " << FLAGS_noise_w * noise << std::endl;
        Eigen::Vector3d noise_v;
        noise_v << noise , noise , noise ;
        auto Pw_noise  = FLAGS_noise_w * noise_v  +  Pw;
        //std::cout << " old pw: \n" << Pw
        //    << " after added noise: \n" << Pw_noise << std::endl;
        Eigen::Vector3d Pc = Rcw * (Pw_noise - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x/z,y/z);
        camera_pose_ft.push_back(camera_pose[i]);
    }
    
    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();

    std::vector< Eigen::Vector2d>  image_uv_v;
    for(auto& p: camera_pose_ft) {
        image_uv_v.push_back(p.uv);
    }
    /* your code begin */
    
    //triangulatePoint( camera_pose_ft[0], camera_pose_ft[1], camera_pose_ft[0].uv, camera_pose_ft[1].uv, P_est);
    multiFrameTriangulateP(camera_pose_ft, image_uv_v, P_est, FLAGS_n_frame);
 
    /* your code end */
    
    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    // TODO:: 请如课程讲解中提到的判断三角化结果好坏的方式，绘制奇异值比值变化曲线
    return 0;
}

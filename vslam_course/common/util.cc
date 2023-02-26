#include "util.h"
#include "camera.h"

namespace zs {

/**
 *  start from left-upper: v1. d1 is distance to upper boundary
 */
template<typename ValType, typename DistType>
ValType bilinear(const ValType& val1, const ValType& val2, 
    const ValType& val3, const ValType& val4,
    const DistType& d1, const DistType& d2, const DistType& d3, const DistType& d4) {
        return val1 * d2 * d3 + val2 * d3 * d4 + val3 * d1 * d4 + val4 * d1 * d2;
    }


zs::Point2D distort(const zs::Point2D& point, const zs::CameraPtr cam) {
    double u = point.x;
    double v = point.y;


    double x = (u - cam->cx) / cam->fx;
    double y = (v - cam->cy) / cam->fy;
    double r = sqrt(x * x + y * y);
    double r2 = r * r;
    double r4 = r2 * r2;
    double coef = 1 + cam->k1 * r2 + cam->k2 * r4;
    double x_distorted = coef * x;
    double y_distorted = coef * y;

    // only consider radical distort
    //double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
    //double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
    double u_distorted = cam->fx * x_distorted + cam->cx;
    double v_distorted = cam->fy * y_distorted + cam->cy;

    return zs::Point2D(u_distorted, v_distorted);
}



cv::Mat undistort(cosnt cv::Mat& img, const zs::CameraPtr cam) {
    int w = img.cols;
    int h = img.rows;

    cv::Mat undistort_mat(h, w, cv::CV_8UC1);
    for(int i=0; i< w; i++) {
        for(int j=0; j< h; j++) {
            zs::Point2D p = distort(zs::Point2D(i, j), cam);
            int u = p.x, v = p.y;
            if(u >= 0 && v >=0 && u < w && v < h) {
                uchar px = img.at<uchar>(v,u);
                if (u + 1 < w && v + 1 < h) {
                    px = ilinear<uchar, double>(img.at<uchar>(v,u),
                                                img.at<uchar>(v, u+1),
                                                img.at<uchar>(v+1,u),
                                                img.at<uchar>(v+1, u+1),
                                                py - v,
                                                1 - (px - u),
                                                1 - (py - v),
                                                p.x - u);
                }

                undistort_img.at<uchar>(j, i) = px;

            }

        }
    }

    return undistort_mat;
}

zs::Point2D project(const zs::Point3D pt_cam, const zs::CameraPtr cam) {
    zs::Point2D p;
    p.x = cam->fx * pt_cam.x / pt_cam.z + cam->cx;
    p.y = cam->fy * pt_cam.y/pt_cam.z  + cam->cy;
    return p;
}

zs::Point2D project(const zs::Point pt_world, const zs::CameraPtr cam, const zs::Pose3D pose)  {
    zs::Point3D pt_cam = transform(pt_world, pose);
    zs::Point2D p = project(pt_cam,  cam);
    return p;
}

zs::Point3D unproject(const zs::Point2D p, zs::CameraPtr cam) {
    // return 
    return zs::Point3D((p.x - cam->cx) / fx, (p.y - cam->cy)/ fy, 1);
}


zs::Point3D unproject(const zs::Point2D p, zs::CameraPtr cam , const zs::Pose3D pose) {
    zs::Point3D pt_cam = unproject(p, cam);
    Eigen::Vector4d p_src;
    p_src << pt_cam.x,
            pt_cam.y,
            pt_cam.z,
            1;

    Eigen::Vector4d p_world = pose.Rt().inverse() * p_src;
    return zs::Point3D(p_world(0), p_world(1), p_world(2));
}


zs::Point3D transform(const zs::Point3D pt, zs::Pose3D pose) {
    Eigen::Vector4d p_src;
    p_src << pt.x ,
            pt.y,
            pt.z,
            1;
    Eigen::Vector4d p_dst = pose.Rt() * p_src;
    return zs::Point3D(p_dst(0), p_dst(1), p_dst(2));
}

}
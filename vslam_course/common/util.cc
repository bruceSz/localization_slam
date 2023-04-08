#include "util.h"
#include "camera.h"

#include <Eigen/Core>
#include <Eigen/LU>
namespace zs {

/**
 *  start from left-upper: v1. d1 is distance to upper boundary, clock-wise.
 */
template<typename ValType, typename DistType>
ValType bilinear(const ValType& val1, const ValType& val2, 
    const ValType& val3, const ValType& val4,
    const DistType& d1, const DistType& d2, const DistType& d3, const DistType& d4) {
        return val1 * d2 * d3 + val2 * d3 * d4 + val3 * d1 * d4 + val4 * d1 * d2;
    }


zs::Point2D distort(const zs::Point2D& point, const Camera::CameraPtr cam) {
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



cv::Mat undistort(const cv::Mat& img, const Camera::CameraPtr cam) {
    int w = img.cols;
    int h = img.rows;

    cv::Mat undistort_mat(h, w, CV_8UC1);
    for(int i=0; i< w; i++) {
        for(int j=0; j< h; j++) {
            // i=x,j =y
            zs::Point2D p = distort(zs::Point2D(i, j), cam);
            int u = p.x;
            int v = p.y;
            if(u >= 0 && v >=0 && u < w && v < h) {
                uchar px = img.at<uchar>(v,u);
                if (u + 1 < w && v + 1 < h) {
                    px = bilinear<uchar, double>(img.at<uchar>(v,u),
                                                img.at<uchar>(v, u+1),
                                                img.at<uchar>(v+1,u),
                                                img.at<uchar>(v+1, u+1),
                                                p.y - v,
                                                1 - (p.x - u),
                                                1 - (p.y - v),
                                                p.x - u);
                }
                // j is row, i is column
                undistort_mat.at<uchar>(j, i) = px;

            }

        }
    }

    return undistort_mat;
}

zs::Point2D project(const zs::Point3D& pt_cam, const Camera::CameraPtr cam) {
    zs::Point2D p;
    p.x = cam->fx * pt_cam.x / pt_cam.z + cam->cx;
    p.y = cam->fy * pt_cam.y/pt_cam.z  + cam->cy;
    return p;
}

zs::Point2D project(const zs::Point3D pt_world, const Camera::CameraPtr cam, const zs::Pose3D pose)  {
    zs::Point3D pt_cam = transform(pt_world, pose);
    zs::Point2D p = project(pt_cam,  cam);
    return p;
}

zs::Point3D unproject(const zs::Point2D p, Camera::CameraPtr cam) {
    // return 
    return zs::Point3D((p.x - cam->cx) / cam->fx, (p.y - cam->cy)/ cam->fy, 1);
}


zs::Point3D unproject(const zs::Point2D p, Camera::CameraPtr cam , const zs::Pose3D pose) {
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

cv::Mat mergeImage(const std::vector<cv::Mat>& imgs, int w, int h) {
    int n = (int)imgs.size();
    int merge_w, merge_h;
    if(n <= 3) {
        merge_w = w * n;
        merge_h = h;
    } else if (n % 2 ==0) {
        merge_w = w * n /2;
        merge_h = h *2;
    } else {
        merge_w = w * n /2 +1;
        merge_h = h *2;
    }

    cv::Mat merge_img(merge_h, merge_w, CV_8UC3);
    for(int i=0; i< n; i++) {
        int row, col;
        if(n <= 3) {
            row = 0;
            col = i;
        } else if(n %2 == 0) {
            row = i< n /2 ? 0: 1;
            col = i % (n/2);
        } else {
            row = i < (n/2 + 1) ? 0 : 1;
            col = i % (n/2 + 1);
        }

        cv::Mat tmp;
        cv::resize(imgs[i], tmp, cv::Size(w,h));

        cv::Mat img_col(h, w, CV_8UC3);
        if(tmp.channels() ==1) {
            std::vector<cv::Mat> channels(3, tmp);
            cv::merge(channels, img_col);
        } else {
            tmp.copyTo(img_col);
        }

        img_col.copyTo(merge_img(cv::Rect(col* w, row * h, w, h)));
    }

    return merge_img;

}


Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> load_matrix(const std::string& file, int row, int col) {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mat;
    mat.resize(row, col);

    std::ifstream fs(file.c_str());
    if (!fs.is_open()) {
        std::cerr << "failed to load file: " << file << std::endl;
        return mat;
    }

    for(int r =0; r< row; r++) {
        for(int c = 0; c< col; c++) {
            fs >> mat(r, c);
        }
    }

    return mat;
}


}
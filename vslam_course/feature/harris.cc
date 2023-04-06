#include "harris.h"
#include "common/math_util.h"
namespace zs {

void Harris::detect(const cv::Mat& src, 
                cv::Mat& resp,
                const int aperture_size,
                const int block_size,
                const double k,
                Harris::ResponseType resp_type,
                cv::BorderTypes border_type) {
    cv::Size size = src.size();
    resp = cv::Mat::zeros(size, resp.type());
    std::cout << "resp type: " << resp.type() << std::endl;

    cv::Mat dx, dy;
    cv::Sobel(src, dx, CV_32F, 1, 0, aperture_size);
    cv::Sobel(src, dy, CV_32F, 0, 1, aperture_size);


    cv::Mat cov(size, CV_32FC3);

    for(int i=0; i< size.height; i++) {
        float* cov_data = cov.ptr<float>(i);
        const float* dx_data = dx.ptr<float>(i);
        const float* dy_data = dy.ptr<float>(i);
        for(int j=0; j< size.width; j++) {
            float _dx = dx_data[j];
            float _dy = dy_data[j];
            cov_data[j*3] = _dx * _dx;
            cov_data[j*3 +1] = _dx * _dy;
            cov_data[j*3 +2] = _dy * _dy;
        }
    }
    // sum up gradient @p(x,y) at blockSize * blockSize window.
    // refer: https://blog.csdn.net/lwzkiller/article/details/54633670
    cv::boxFilter(cov, cov, cov.depth(), cv::Size(block_size, block_size), cv::Point(-1,-1), false);

    cv::Mat _resp = cv::Mat::zeros(size, CV_32FC1);
    cv::Mat _resp_norm;
    if(resp_type == HARRIS) {
        calcHarris(cov, _resp, k);
    } else  if( resp_type == MINEIGENVAL) {
        calcMinEigenVal(cov, _resp);
    }
    //cv::cornerHarris(src, _resp, blockSize, aperture_size, k);

    cv::normalize(_resp, _resp_norm, 0, 255, cv::NORM_MINMAX);
    cv::convertScaleAbs(_resp_norm, resp);
}

/**
 * @param  cov is the gray gradient @ mat(x,y), 
 * 3 channels with  Ix^2, Ix*Iy , Iy^2
 * @param resp is the resp value @ mat(x,y), where:
 *    Resp = det(M) - k(trace(M))^2
 *    and det(M) = a*c - b*b
 *    
 * @param k is the parameter in above formular
 * */
void Harris::calcHarris(const cv::Mat& cov, cv::Mat& resp, const double k) {
    
    cv::Size size = cov.size();

    resp = cv::Mat::zeros(size, resp.type());

    if(cov.isContinuous() && resp.isContinuous()) {
        size.width *= size.height;
        size.height = 1;
    }

    for(int i=0; i< size.height; i++) {
        const float* conv_data = cov.ptr<float>(i);
        float* resp_data = resp.ptr<float>(i);
        for(int j=0; j< size.width; j++) {
            float a = conv_data[j*3];
            float b = conv_data[j*3 + 1];
            float c = conv_data[j*3 + 2];
            resp_data[j] = (float)(a * c - b * b - k * (a+c)*(a+c));
        }
    }
}   

/*
* @param cov grayscale gradient @p(x,y) , 3 channels represent:
*    Ix^2, Ix*Iy , Iy^2
* @param resp
*/
void calcMinEigenVal(const cv::Mat& cov, cv::Mat& resp) {
    cv::Size sz = cov.size();
    resp = cv::Mat::zeros(sz, resp.type());
    if(cov.isContinuous() && resp.isContinuous()) {
        sz.width *= sz.height;
        sz.height = 1;
    }


    for(int i=0; i<sz.height; ++i) {
        const float* cov_data = cov.ptr<float>(i);
        float * resp_data = resp.ptr<float>(i);
        for(int j=0; j< sz.width; j++) {
            float a = cov_data[j*3] * 0.5f;
            float b = cov_data[j*3+1] ;
            float c = cov_data[j*3+2] * 0.5;
            //TODO: understand this .
            resp_data[j]  = (a+c) - std::sqrt((a-c)*(a-c) + b*b);
        }
    }
}

void getKeyPoints(const cv::Mat& resp, std::vector<cv::Point2i>& kps, 
                    const double resp_threshold, const int nms_sup_radius = 8) {
    cv::Mat _resp = resp.clone();
    // current max resp value in current round.
    double curr_resp = 0.f;

    while(1) {
        double min_v;
        cv::Point min_idx, max_idx;
        cv::minMaxLoc(_resp, &min_v, &curr_resp, &min_idx, &max_idx);
        
        if(curr_resp < resp_threshold) 
            break;
        kps.push_back(max_idx);
        // set _resp value to 0 in nms_radius covered ranges.
        for(int j= -nms_sup_radius; j<=nms_sup_radius; j++) {
            for( int k=-nms_sup_radius; k <= nms_sup_radius; k++) {
                int r = max_idx.y + j;
                int c = max_idx.x + k;
                if(r >=0 && r < _resp.rows &&
                    c >=0 && c < _resp.cols) {
                        _resp.at<uchar>(r,c) = 0;
                    }

            }
        }
    }
    
}

cv::Mat Harris::plotMatchOneImage(const cv::Mat& query, const std::vector<cv::Point2i>& reference_kps, const std::vector<cv::Point2i>& query_kps, const std::vector<int>& match_) {
    cv::Mat img_res(query.rows, query.cols, CV_8UC3);
    if(query.channels() ==1) {
        std::vector<cv::Mat> channels(3, query);
        cv::merge(channels, img_res);
    }  else {
        query.copyTo(img_res);
    }
}

 void Harris::match(const std::vector<std::vector<uchar>>& reference_desc, const std::vector<std::vector<uchar>>& query_desc, 
                    std::vector<int>& match_, const double lambda) {
    int num_kp = (int)query_desc.size();
    std::vector<double> ssd_vec(num_kp,0);

    match_.clear();
    match_.resize(num_kp,-1);

    double g_min_sdd = std::numeric_limits<double>::max();

    for(int i=0; i<num_kp; i++ ) {
        double min_sdd = std::numeric_limits<double>::max();
        int match_idx = -1;
        for(size_t j =0 ; j< reference_desc.size(); j++ ) {
            double ssd = zs::ssd<uchar>(query_desc[i], reference_desc[j]);
            if(ssd < min_sdd) {
                min_sdd = ssd;
                match_idx = j;
                if(min_sdd > 0 && min_sdd < g_min_sdd) {
                    g_min_sdd = min_sdd;
                }
            }
        }
        ssd_vec[i] = g_min_sdd;
        match_[i] = match_idx;
    }

    // ratio here.
    g_min_sdd *= lambda;
    for(int i=0; i<num_kp; i++ ) {
        if (ssd_vec[i] >= g_min_sdd) {
            match_[i] = -1;
        }
    }


 }


void getDescriptors(const cv::Mat& src, const std::vector<cv::Point2i>& kps, 
                    std::vector<std::vector<uchar>>& descs, const int r) {
    int num_kps = kps.size();
    descs.clear();
    descs.resize(num_kps);

    for(int i=0; i<num_kps; i++) {
        descs[i].resize((2*r+1)*(2*r+1));
    }


    for (int i=0; i< num_kps; i++) {
        int idx = 0;
        for(int j=-r; j<=r; j++) {
            for(int k=-r;k<=r;k++) {
                int row = kps[i].y + j;
                int col = kps[i].x + k;
                if(row>=0 && row < src.rows &&
                    col >=0 && col < src.cols ) {
                        descs[i][idx] = src.at<uchar>(row,col);
                    } else {
                        descs[i][idx] =0;
                    }
                    idx++;
            }
        }
    }
}
    


} // namespace harris
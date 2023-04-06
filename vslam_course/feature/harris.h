
#pragma once
#include <opencv2/opencv.hpp>



namespace zs {

class Harris {
public:

    enum ResponseType {
        HARRIS, 
        MINEIGENVAL
    };

    Harris();
    ~Harris();

    /**
     * 
     * @param src
     * @param resp           corner response matrix
     * @param aperture_size  sobel size
     * @param block_size     adjancy block size
     * @param k              magic number
     * @param resp_typle     
     * @param border_type
     * */

    void detect(const cv::Mat& src, 
                cv::Mat& resp,
                const int aperture_size,
                const int block_size,
                const double k,
                Harris::ResponseType resp_type = HARRIS,
                cv::BorderTypes border_type = cv::BORDER_DEFAULT);

    /**
     * @param resp
     * @param kps   
     * @param resp_threshold
     * @param nms_sup_radius
     */
    void getKeyPoints(const cv::Mat& resp, std::vector<cv::Point2i>& kps, 
                    const double resp_threshold, const int nonmaximum_sup_radius = 8);
    
    /**
     * @param src        source image
     * @param kps        kp vector
     * @param descs      descriptor for each kp.
     * @param r          radius
     */ 
    void getDescriptors(const cv::Mat& src, const std::vector<cv::Point2i>& kps, 
                            std::vector<std::vector<uchar>>& descriptors, const int r);
    
    void calcHarris(const cv::Mat& cov, cv::Mat& resp, const double k);
    
    void calcMinEigenVal(const cv::Mat& cov, cv::Mat& resp);

    void match(const std::vector<std::vector<uchar>>& reference_desc, const std::vector<std::vector<uchar>>& query_desc, std::vector<int>& match_, const double lambda);
    cv::Mat plotMatchOneImage(const cv::Mat& query, const std::vector<cv::Point2i>& reference_kps, const std::vector<cv::Point2i>& query_kps, const std::vector<int>& match_);

};

} // namespace zs

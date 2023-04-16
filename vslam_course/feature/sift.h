/**
 * 
 * 1 extract pyramid 
 * 2 do gaussian filter for each scale.
 * 3 compute dog
 * 4 compute extreme point 
 * 5 compute hog descriptor
 * 6 match with descriptor.
 * */


#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>


namespace zs {

class Sift {
  public:
    /**
     * @param nfts
     * @param octaves
     * @param octave_scales
     * @param constant_threshold
     * @param edge_threshold
     * @param sigma
     */

    Sift(n nfts, int octaves =5, int octave_scales=3, int constant_threshold=0.04, 
        double edge_threshold=10, double sigma=1.6);

    ~Sift();

    void detect(const cv::Mat& src, std::vector<cv::KeyPoint>& kps, std::vector<std::vector<float>> descs);

    void match(const std::vector<std::vector<float>>& ref_desc, const std::vector<std::vector<float>>& query_descs, std::vector<int>& match);

    
};

}
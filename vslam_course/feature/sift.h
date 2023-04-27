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

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace zs {

struct KeyPointResponseCompare {
  inline bool operator()(const cv::KeyPoint& kp1,
                         const cv::KeyPoint& kp2) const {
    return kp1.response > kp2.response;
  }
};

struct KeypointResponseGreaterThanOrEqualToThreshold
{
    KeypointResponseGreaterThanOrEqualToThreshold(float _value) :
    value(_value)
    {
    }
    inline bool operator()(const cv::KeyPoint& kpt) const
    {
        return kpt.response >= value;
    }
    float value;
};

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

  Sift(int nfts, int octaves = 5, int octave_scales = 3,
       int constant_threshold = 0.04, double edge_threshold = 10,
       double sigma = 1.6);

  ~Sift();

  cv::Mat createInitialImage(const cv::Mat& img);

  void detect(const cv::Mat& src, std::vector<cv::KeyPoint>& kps,
              std::vector<std::vector<float>> descs);

  void match(const std::vector<std::vector<float>>& ref_desc,
             const std::vector<std::vector<float>>& query_descs,
             std::vector<int>& match);

  void plotMatchImages(const cv::Mat& ref_img, const cv::Mat& query_img,
                       const std::vector<int>& match,
                       const std::string& saved_path, bool to_save);

  void plotGaussignPyramid(const std::string& saved_path, bool to_save);

  void plotDogPyramid(const std::string& saved_path, bool to_save);

  void plotKeypoints(const std::string& saved_path,
                     std::vector<cv::KeyPoint>& kps, bool to_save);

 private:
  void buildGaussianPyramid(const cv::Mat& base, std::vector<cv::Mat>& pyramid);

  cv::Mat credateInitImage(const cv::Mat& base);

  void buildDogPyramid(const std::vector<cv::Mat>& gaussian_pyr,
                       std::vector<cv::Mat>& dog_pyr);

  void findScaleSpaceExtrema(const std::vector<cv::Mat>& dog_pyr,
                             std::vector<cv::KeyPoint> kps);

  void retainBest(std::vector<cv::KeyPoint>& kps, int n_points);

  void resumeScale(std::vector<cv::KeyPoint>& kps);

  void calcDescriptrs(const std::vector<cv::Mat>& gaussian_pyr,
                      const std::vector<cv::KeyPoint>& kps,
                      std::vector<std::vector<float>>& descs);

  void calcPatchDescriptors(const cv::Mat& patch_norm, const cv::Mat& path_dir,
                            std::vector<float>& desc);

  void calcCellHog(const cv::Mat& cell_norm, const cv::Mat& cell_dir,
                   std::vector<float>& hog);

  void plotPyramid(const std::vector<cv::Mat>& pyr, int width, int height,
                   int n, const std::string& save_path, bool to_save);

  int nfts_;

  int octaves_;

  int octave_scales_;

  double contrast_threshold_;
  double edge_threshold_;
  double sigma_;
  std::vector<cv::Mat> gaussian_pyr_;
  std::vector<cv::Mat> dog_pyr_;
};

}  // namespace zs
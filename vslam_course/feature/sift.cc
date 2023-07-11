#include "sift.h"

#include <iostream>
#include <algorithm>

#include "common/draw_utils.h"
#include "common/util.h"
#include "common/math_util.h"

namespace zs {

  Sift::Sift(int nfts, int octaves, int octave_scales, int contrast_threshold,
    double edge_threshold, double sigma)
    : sigma_(sigma),
    contrast_threshold_(contrast_threshold),
    octave_scales_(octave_scales),
    octaves_(octaves),
    nfts_(nfts) {
    std::cout << "Sift constructor" << std::endl;
  }

  Sift::~Sift() {}

  cv::Mat Sift::createInitialImage(const cv::Mat& img) {
    cv::Mat upper_img, base;
    cv::resize(img, upper_img, cv::Size(img.cols * 2, img.rows * 2), 0, 0,
      cv::INTER_LINEAR);
    float sig_diff =
      std::pow(std::max(sigma_ * sigma_ - 0.5 * 0.5 * 4, 0.01), 0.5);
    std::cout << "sig_diff: " << sig_diff << std::endl;

    cv::GaussianBlur(upper_img, base, cv::Size(), sig_diff, sig_diff);
    return base;
  }

  void Sift::detect(const cv::Mat& src, std::vector<cv::KeyPoint>& kps,
    std::vector<std::vector<float>> descs) {
    cv::Mat base = createInitialImage(src);
    buildGaussianPyramid(base, gaussian_pyr_);
    buildDogPyramid(base, dog_pyr_);

    findScaleSpaceExtrema(dog_pyr_, kps);

    std::cout << "findScaleSpaceExtrema" << std::endl;
  }

  void Sift::buildGaussianPyramid(const cv::Mat& base,
    std::vector<cv::Mat>& pyr) {
    pyr.clear();
    pyr.resize(octaves_ * (octave_scales_ + 3));

    std::vector<double> sig(octave_scales_ + 3);

    sig[0] = sigma_;
    double k = std::pow(2.0, 1.0 / octave_scales_);
    for (int i = 1; i < octave_scales_; i++) {
      double sig_prev = std::pow(k, (double)(i - 1)) * sigma_;
      double sig_total = sig_prev * k;
      sig[i] = std::sqrt(sig_total * sig_total - sig_prev * sig_prev);
    }

    for (int o = 0; o < octaves_; o++) {
      for (int i = 0; i < octave_scales_; i++) {
        cv::Mat& dst = pyr[o * (octave_scales_ + 3) + i];
        if (o == 0 && i == 0) {
          dst = base;
        }
        else if (i == 0) {
          const cv::Mat& src =
            pyr[(o - 1) * (octave_scales_ + 3) + octave_scales_];
          cv::resize(src, dst, cv::Size(src.cols / 2, src.rows / 2), 0, 0,
            cv::INTER_LINEAR);
        }
        else {
          const cv::Mat& src = pyr[(o) * (octave_scales_ + 3) + i - 1];
          cv::GaussianBlur(src, dst, cv::Size(), sig[i], sig[i]);
        }
      }
    }
    return;
  }

  void Sift::buildDogPyramid(const std::vector<cv::Mat>& gaussian_pyr,
    std::vector<cv::Mat>& dog_pyr) {
    dog_pyr.clear();
    dog_pyr.resize(octaves_ * (octave_scales_ + 2));
    for (int idx = 0; idx < int(dog_pyr.size()); idx++) {
      int o = idx / (octave_scales_ + 2);
      int i = idx % (octave_scales_ + 2);

      const cv::Mat& src1 = gaussian_pyr[o * (octave_scales_ + 3) + i];
      const cv::Mat& src2 = gaussian_pyr[o * (octave_scales_ + 3) + i + 1];
      cv::Mat& dst = dog_pyr[idx];
      cv::subtract(src2, src1, dst, cv::noArray());
      cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);
    }
  }

  bool Sift::isNearByExtrema(const cv::Mat& img, int r, int c, float v) {
    //CHECK(r >= 1 && c >= 1 && r < mat.rows - 1 && c < mat.cols - 1);
    // float val = mat.at<float>(r, c);

    float _00, _01, _02;
    float _10, _12;
    float _20, _21, _22;

    _00 = img.at<float>(r - 1, c - 1);
    _01 = img.at<float>(r - 1, c);
    _02 = img.at<float>(r - 1, c + 1);
    _10 = img.at<float>(r, c - 1);
    _12 = img.at<float>(r, c + 1);
    _20 = img.at<float>(r + 1, c - 1);
    _21 = img.at<float>(r + 1, c);
    _22 = img.at<float>(r + 1, c + 1);

    std::vector<float> near_dog = { _00, _01, _02, _10, _12, _20, _21, _22 };

    float vmax = *std::max_element(near_dog.begin(), near_dog.end());
    float vmin = *std::min_element(near_dog.begin(), near_dog.end());

    if (v > 0 && v > vmax) {
      return true;
    }

    if (v < 0 && v < vmin) {
      return true;
    }

    return false;
  }

  void Sift::findScaleSpaceExtrema(const std::vector<cv::Mat>& dog_pyr,
    std::vector<cv::KeyPoint>& keypoints) {
    int border = 8;
    for (int o = 0; o < octaves_; o++) {
      for (int i = 0; i < octave_scales_; i++) {
        int idx = o * (octave_scales_ + 2) + i;
        const cv::Mat& img = dog_pyr[idx];
        const cv::Mat& prev = dog_pyr[idx - 1];
        const cv::Mat& next = dog_pyr[idx + 1];

        for (int r = border; r < img.rows - border; r++) {
          for (int c = border; c < img.cols - border; c++) {
            float val = img.at<float>(r, c);
            if (std::abs(val) < contrast_threshold_) {
              continue;
            }
            bool valid = false;
            // is extrema at current scale.
            if (isNearByExtrema(img, r, c, val)) {
              // is extrema at previous scale. (higher resolution/scale)
              if (isNearByExtrema(prev, r, c, val)) {
                // is extrema at next scale. (lower resolution/scale)
                if (isNearByExtrema(next, r, c, val)) {
                  float prev_v = prev.at<float>(r, c);
                  float next_v = next.at<float>(r, c);
                  if (val > 0) {
                    valid = (val >= std::max(prev_v, next_v));
                  }
                  else {
                    valid = (val <= std::min(prev_v, next_v));
                  }
                }
              }
            }
            if (valid) {
              cv::KeyPoint kpt;
              // transform coordinates to bottom of the pyramid.
              kpt.pt.x = c * (1 << o);
              kpt.pt.y = r * (1 << o);
              // figure it out
              kpt.octave = o + (i << 8);
              kpt.response = val;
              kpt.size = sigma_ * powf(2.f, i / octave_scales_) * (1 << o) * 2;
              keypoints.push_back(cv::KeyPoint(r, c, val));
            }
          }
        }
      }
    }
  }

  void retainBest(std::vector<cv::KeyPoint>& kpts, int n_pts) {
    if (n_pts >= 0 && n_pts < kpts.size()) {
      if (n_pts == 0) {
        kpts.clear();
        return;
      }

      std::nth_element(kpts.begin(), kpts.begin() + n_pts - 1, kpts.end(),
        KeyPointResponseCompare());
      float threshold_val = kpts[n_pts - 1].response;
      // get those with equal response (compared with threshold)
      std::vector<cv::KeyPoint>::const_iterator new_end =
        std::partition(kpts.begin() + n_pts, kpts.end(),
          KeypointResponseGreaterThanOrEqual(threshold_val));
      kpts.resize(new_end - kpts.begin());
    }
  }

  void Sift::resumeScale(std::vector<cv::KeyPoint>& kpts) {
    for (size_t i = 0; i < kpts.size(); i++) {
      cv::KeyPoint& kpt = kpts[i];
      float scale = 0.5;

      kpt.pt *= scale;
      kpt.size *= scale;
    }
  }

  void Sift::calcDescriptrs(const std::vector<cv::Mat>& gaussian_pyr,
    const std::vector<cv::KeyPoint>& kps,
    std::vector<std::vector<float>>& desc) {
    desc.clear();
    desc.resize(kps.size());

    int n = (int)gaussian_pyr.size();

    std::vector<cv::Mat> pyr_dx(n);
    std::vector<cv::Mat> pyr_dy(n);
    std::vector<cv::Mat> pyr_norm(n);
    std::vector<cv::Mat> pyr_dir(n);

    // compute gradient norm and direction for each point.
    for (int i = 0; i < n; i++) {
      cv::Sobel(gaussian_pyr[i], pyr_dx[i], CV_32F, 1, 0, 3);
      cv::Sobel(gaussian_pyr[i], pyr_dy[i], CV_32F, 0, 1, 3);

      pyr_norm[i] = cv::Mat::zeros(gaussian_pyr[i].size(), CV_32FC1);
      pyr_dir[i] = cv::Mat::zeros(gaussian_pyr[i].size(), CV_32FC1);

      for (int r = 0; r < gaussian_pyr[i].rows; r++) {
        for (int c = 0; c < gaussian_pyr[i].cols; c++) {
          pyr_norm[i].at<float>(r, c) =
            std::sqrt(std::pow(pyr_dx[i].at<float>(r, c), 2) +
              std::pow(pyr_dy[i].at<float>(r, c), 2));

          if (pyr_dx[i].at<float>(r, c) == 0 && pyr_dy[i].at<float>(r, c) == 0) {
            pyr_dir[i].at<float>(r, c) = std::numeric_limits<float>::max();
          }
          else {
            pyr_dir[i].at<float>(r, c) =
              std::atan2(pyr_dy[i].at<float>(r, c), pyr_dx[i].at<float>(r, c));
          }
        }
      }
    }

    // for each keypoint, compute its descriptor.
    // 1. 4 * 4 patch cells,
    // 2. group norms into 8 bin.

    for (int i = 0; i < (int)kps.size(); i++) {
      const cv::KeyPoint& kpt = kps[i];
      // get low 8 bit value.
      int o = kpt.octave & 255;

      int layer = (kpt.octave >> 8) & 255;
      std::cout << "o: " << o << " layer: " << layer << std::endl;

      const cv::Mat& img_norm = pyr_norm[o * (octave_scales_ + 3) + layer];
      const cv::Mat& img_dir = pyr_dir[o * (octave_scales_ + 3) + layer];

      int r, c;
      //?
      if (o == 0) {
        r = kpt.pt.y * 2;
        c = kpt.pt.x * 2;
      }
      else {
        r = kpt.pt.y / (1 << (o - 1));
        c = kpt.pt.x / (1 << (o - 1));
      }
      std::cout << "img size: " << img_norm.size() << std::endl;
      std::cout << "kp: " << kpt.pt << std::endl;

      cv::Mat patch_norm =
        img_norm.rowRange(r - 7, r + 9).colRange(c - 7, c + 9).clone();
      cv::Mat patch_dir =
        img_dir.rowRange(r - 7, r + 9).colRange(c - 7, c + 9).clone();
      std::vector<float> kp_desc;
      std::cout << "#1.2" << std::endl;

      calcPatchDesc(patch_norm, patch_dir, kp_desc);
      desc[i] = kp_desc;
    }
  }

  void Sift::calcPatchDesc(const cv::Mat& patch_norm, const cv::Mat& patch_dir,
    std::vector<float>& desc) {
    std::cout << "#2" << std::endl;
    assert(patch_norm.rows == 16 && patch_norm.cols == 16);
    assert(patch_dir.rows == 16 && patch_dir.cols == 16);
    std::cout << "#2.1" << std::endl;

    desc.clear();
    desc.reserve(128);

    cv::Mat patch_norm_gaussian;
    cv::GaussianBlur(patch_norm, patch_norm_gaussian, cv::Size(), 1.5 * 16,
      1.5 * 16);

    for (int i = 0; i < 16; i++) {
      cv::Mat cell_norm =
        patch_norm_gaussian.rowRange((i / 4) * 4, (i / 4) * 4 + 4)
        .colRange((i % 4) * 4, (i % 4) * 4 + 4)
        .clone();
      cv::Mat cell_dir = patch_dir.rowRange((i / 4) * 4, (i / 4) * 4 + 4)
        .colRange((i % 4) * 4, (i % 4) * 4 + 4)
        .clone();

      std::vector<float> hog;
      calcCellHog(cell_norm, cell_dir, hog);
      desc.insert(desc.end(), hog.begin(), hog.end());
    }
    // 16*8 == 128

    assert(desc.size() == 128);
    float sum_square = 0.0f;
    for (int i = 0; i < desc.size(); i++) {
      sum_square += desc[i] * desc[i];
    }

    if (sum_square != 0) {
      for (int i = 0; i < desc.size(); i++) {
        desc[i] /= std::sqrt(sum_square);
      }
    }
  }

  void Sift::plotDogPyramid(const std::string& saved_path, bool to_save) {
    if (dog_pyr_.empty()) {
      return;
    }

    std::vector<cv::Mat> dog_pyr_norm(dog_pyr_.size());
    for(int i=0; i< dog_pyr_.size(); i++) {
      cv::normalize(dog_pyr_[i], dog_pyr_norm[i], 0, 255, cv::NORM_MINMAX);
    }
    int w = dog_pyr_norm[0].cols, h = dog_pyr_norm[0].rows;
    int n = dog_pyr_norm.size() / octaves_;
    plotPyramid(dog_pyr_norm, w/5, h/5,n, saved_path, to_save);

  }

  void Sift::plotGaussianPyramid(const std::string& saved_path, bool to_save) {
    if (gaussian_pyr_.empty()) {
      return;
    }

    int w = gaussian_pyr_[0].cols, h = gaussian_pyr_[0].rows;
    plotPyramid(gaussian_pyr_, w/5, h/5, octaves_, saved_path, to_save);
  }

void Sift::plotPyramid(const std::vector<cv::Mat>& pyr, int width, int height,
                   int n, const std::string& save_path, bool to_save)
   {
      int octave = pyr.size()/ n;
      cv::Mat pyr_img = cv::Mat::zeros((2-std::pow(2, -(octave-1))) * height, n * width,  CV_8UC1);

      int w = width, h = height;
      for(int o=0; o< octave; o++) {
        for(int i=0; i<n; i++) {
          const cv::Mat& img = pyr[o*n + i];
          cv::Mat tmp;
          cv::resize(img, tmp, cv::Size(w,h));
          tmp.copyTo(pyr_img(cv::Rect(w*i, (2-std::pow(2, -(o-1)))* height, w, h )));
        }
        w *= 0.5;
        h *= 0.5;
      }

      if (to_save) {
        cv::imwrite(save_path, pyr_img);
      }

      cv::imshow("pyramid", pyr_img);
      cv::waitKey(-1);
      cv::destroyWindow("pyramid");



    }

void Sift::match(const std::vector<std::vector<float>>& ref_descs, const std::vector<std::vector<float>>& query_descs, std::vector<int>& match) {
              int num_kp = (int) query_descs.size();
              std::vector<double> ssd_vec(num_kp, 0);


              match.clear();
              match.resize(num_kp,-1);

              double global_min = std::numeric_limits<double>::max();
              for(int i=0; i< num_kp; i++) {
                double min_ssd = std::numeric_limits<double>::max();
                int match_idx = -1;
                for (int j=0; j<ref_descs.size(); j++) {
                  double ssd = zs::ssd(query_descs[i], ref_descs[j]);
                  if (ssd < min_ssd) {
                    min_ssd = ssd;
                    match_idx = j;
                    if (min_ssd > 0 && min_ssd < global_min) {
                      global_min = min_ssd;
                    }
                  }
                }

                ssd_vec[i] = min_ssd;
                match[i]= match_idx;
              }

              global_min *= 1.1;

              for(int i=0; i< num_kp; i++) {
                if(ssd_vec[i] > global_min) {
                  match[i] = -1;
                }
              }
              return;
              }
  void Sift::plotMatchTwoImage(const cv::Mat& reference_img, const cv::Mat& query_img,
     const std::vector<cv::KeyPoint>& reference_kps, const std::vector<cv::KeyPoint>& query_kps, 
      const std::vector<int>& match, const std::string& saved_path, bool saved) {
    cv::Mat img_kp_ref = zs::drawKeyPoint(reference_img, reference_kps, cv::Scalar(0, 0, 255));
    cv::Mat img_kp_query = zs::drawKeyPoint(query_img, query_kps, cv::Scalar(0, 0, 255));

    cv::Mat merge_img = zs::mergeImage(std::vector<cv::Mat>{img_kp_ref, img_kp_query}, img_kp_query.cols, img_kp_query.rows);
    for(int i=0; i< (int)match.size(); i++) {
      if (match[i] == -1) {
        continue;
      }
      cv::Point2f pt_ref, pt_query;
      pt_ref.x = reference_kps[match[i]].pt.x;
      pt_ref.y = reference_kps[match[i]].pt.y;
      pt_query.x = query_kps[i].pt.x + reference_img.cols;
      pt_query.y = query_kps[i].pt.y;
      cv::line(merge_img, pt_ref, pt_query, cv::Scalar(0,255, 0));

     }
     if (saved) {
      cv::imwrite(saved_path, merge_img);
     }

     cv::imshow("match", merge_img);
     cv::waitKey(-1);
     cv::destroyWindow("match");
  }


  void Sift::plotKeypoints(const cv::Mat& src,
    std::vector<cv::KeyPoint>& kps, const std::string& saved_path, bool to_save) {

    std::cout << kps.size() << std::endl;
    for (int i = 0; i < kps.size(); i++) {
      std::cout << kps[i].pt.x << "," << kps[i].pt.y << "," << kps[i].size << std::endl;
    }

    cv::Mat img_kp = zs::drawKeyPoint(src, kps, cv::Scalar(0, 0, 255));
    if (to_save) {
      cv::imwrite(saved_path, img_kp);
    }

    cv::imshow("kp", img_kp);
    cv::waitKey(-1);
    cv::destroyWindow("kp");

  }

  void Sift::calcCellHog(const cv::Mat& cell_norm, const cv::Mat& cell_dir, std::vector<float>& hog) {
    assert(cell_norm.rows == 4 && cell_norm.cols == 4);
    assert(cell_dir.rows == 4 && cell_dir.cols == 4);

    hog.clear();
    hog.resize(8);

    for (int i = 0; i < 16; i++) {
      int r = i / 4, c = i % 4;
      // ignore invalid direction(no x change, atan2 is infinite)   
      if (cell_dir.at<float>(r, c) == std::numeric_limits<float>::max()) {
        continue;
      }

      if (cell_dir.at<float>(r, c) >= 0) {
        // we have 8 bins, each bin is 45 degree. -180 ~ +180
        hog[cell_dir.at<float>(r, c) / (M_PI / 4)] += cell_norm.at<float>(r, c);
    } else {
      hog[cell_dir.at<float>(r,c)/(M_PI/4)+7] +=cell_norm.at<float>(r,c);
    }
  }
}

}  // namespace zs
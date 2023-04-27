#include "sift.h"

#include <iostream>

namespace zs {

Sift::Sift(int nfts, int octaves, int octave_scales, int contrast_threshold,
           double edge_threshold, double sigma): sigma_(sigma), 
           contrast_threshold_(contrast_threshold),
           octave_scales_(octave_scales), octaves_(octaves), nfts_(nfts) {
  std::cout << "Sift constructor" << std::endl;
}

Sift::~Sift() {}

cv::Mat Sift::createInitialImage(const cv::Mat& img) {
    cv::Mat upper_img, base;
    cv::resize(img, upper_img, cv::Size(img.cols*2, img.rows*2) , 0, 0, cv::INTER_LINEAR);
    float sig_diff = std::pow(std::max(sigma_ * sigma_ - 0.5*0.5 * 4, 0.01 ), 0.5);
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

void Sift::buildGaussianPyramid(const cv::Mat& base, std::vector<cv::Mat>& pyr) {
    pyr.clear();
    pyr.resize(octaves_ * (octave_scales_ + 3));

    std::vector<double> sig(octave_scales_ + 3);

    sig[0] = sigma_;
    double k = std::pow(2.0, 1.0/octave_scales_);
    for(int i=1; i< octave_scales_; i++) {
        double sig_prev = std::pow(k, (double)(i-1)) * sigma_;
        double sig_total = sig_prev * k;
        sig[i] = std::sqrt(sig_total * sig_total - sig_prev * sig_prev);
    }

    for(int o=0; o < octaves_; o++) {
        for( int i=0; i < octave_scales_; i++) {
            cv::Mat& dst = pyr[o * (octave_scales_ + 3) + i];
            if(o==0 && i==0) {
                dst = base;
            } else if( i == 0) {
                const cv::Mat& src = pyr[(o-1)*(octave_scales_ + 3) + octave_scales_];
                cv::resize(src, dst, cv::Size(src.cols/2, src.rows/2), 0, 0, INTER_LINEAR);
            } else {
                const cv::Mat& src = pyr[(o)*(octave_scales_ + 3) + i-1];
                cv::GaussianBlur(src, dst, cv::Size(), sig[i], sig[i]);
            }


        }

    }
    return;
}

void Sift::buildDogPyramid(const std::vector<cv::Mat>& gaussian_pyr, std::vector<cv::Mat>& dog_pyr) {
    dog_pyr.clear();
    dog_pyr.resize(octaves_ * (octave_scales_ + 2));
    for(int idx = 0; idex < int(dog_pyr.size()); idx++) {
       int o = idx/(octave_scales_ + 2);
       int i = idx%(octave_scales_ + 2);

       const cv::Mat& src1 = gaussian_pyr[o* (octave_scales_ + 3) + i];
       const cv::Mat& src2 = gaussian_pyr[o* (octave_scales_ + 3) + i + 1];
       cv::Mat& dst = dog_pyr[idx];
       cv::substract(src2, src1, dst, cv::noArray());
       cv::normalize(dst, dst, 0, 255, cv::NORM_MINMAX);
    }
}

bool isNearByExtrema(const cv::Mat& mat, int r, int c, float v) {
    CHECK(r >= 1 && c >= 1 && r < mat.rows-1 && c < mat.cols -1);
    //float val = mat.at<float>(r, c);
    
    float _00, _01, _02;
    float _10,    , _12;
    float _20, _21, _22;

    _00 = img.at<float>(r-1, c-1); _01 = img.at<float>(r-1, c); _02 = img.at<float>(r-1,c+1);
    _10 = img.at<float>(r, c-1);                                _12 = img.at<float>(r,c+1);
    _20 = img.at<float>(r+1, c-1); _21 = img.at<float>(r+1, c); _22 = img.at<float>(r+1,c+1);

    std::vector<float> near_dog = {_00, _01, _02, _10,  _12, _20, _21, _22};

    float vmax = std::max_element(near_dog.begin(), near_dog.end());
    float vmin = std::min_element(near_dog.begin(), near_dog.end());

    if (v > 0 && v > vmax) {
        return true;
    }

    if (v < 0 && v < vmin) {
        return true;
    }

    return false;
}


void Sift::findScaleSpaceExtrema(const std::vector<cv::Mat>& dog_pyr, std::vector<cv::KeyPoint> & keypoints) {
    int border = 8;
    for(int o=0; o < octaves_; o++) {
        for(int i=0; i < octave_scales_; i++) {
            int idx = o * (octave_scales_ + 2) + i;
            const cv::Mat& img = dog_pyr[idx];
            const cv::Mat& prev = dog_pyr[idx-1];
            const cv::Mat& next = dog_pyr[idx+1];

            for(int r=border; r < img.rows-border; r++) {
                for(int c = border; c< img.cols-border; c++) {
                    float val = img.at<float>(r, c);
                    if(std::abs(val) < contrast_threshold_) {
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
                                if ( v> 0) {
                                    valid = (val >= std::max(prev_v, next_v));
                                } else {
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
                        kpt.octave = o + (i <<8);
                        kpt.response = val;
                        kpt.size = sigma_ * powf(2.f, i/octave_scales_) * (1 << o) * 2;
                        keypoints.push_back(cv::KeyPoint(r, c, val));
                    }
                }
            }
        }
    }
}

void retainBest(std::vector<cv::KeyPoint>& kpts, int n_pts ) {
    if ( n_pts >=0 && n_pts < kpts.size() ) {
        if (n_pts ==0) {
            kpts.clear();
            return;
        }

        std::nth_element(kpts.begin(), kpts.begin() + n_pts-1, kpts.end(), KeyPointResponseCompare());
        float threshold_val = kpts[n_pts-1].response;
       // get those with equal response (compared with threshold)
        std::vector<cv::KeyPoint>::const_iterator new_end = std::partition(kpts.begin() + n_pts, kpts.end(), KeypointResponseGreaterThanOrEqual(threshold_val));
        kps.resize(new_end - kpts.begin());
    }
}


void Sift::resumeScale(std:vector<cv::KeyPoint>& kpts) {
    for(size_t i = 0; i < kpts.size(); i++) {
        cv::KeyPoint& kpt = kpts[i];
        float scale = 0.5;

        kpt.pt *= scale;
        kpt.size *= scale;
    }
}

}  // namespace zs
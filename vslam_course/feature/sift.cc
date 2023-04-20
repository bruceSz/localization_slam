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

}  // namespace zs
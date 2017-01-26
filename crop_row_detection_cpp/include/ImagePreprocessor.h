//
// Created by noflip on 21/11/16.
//
#include <crop_row_detection.h>

#ifndef NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H
#define NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H

namespace crd_cpp{
    class ImagePreprocessor{
        public:
            ImagePreprocessor (std::string images_path, cv::Size target_size);
            std::vector<cv::Mat> process();
            std::map<period_type, std::vector<phase_type>> get_Xs(const period_type m_mind, const int n_periods, const double m_dstep);
            // std::map<period_type, std::vector<phase_type>> get_Xs(const period_type m_mind, const size_t n_periods, const double m_dstep) {
        private:
            cv::Size m_size;
            std::string m_images_folder = "";
            cv::Mat convertToExG(cv::Mat&);
    };
}

#endif //NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H

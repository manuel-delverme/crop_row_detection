#ifndef NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H
#define NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H

namespace crd_cpp{
    class ImagePreprocessor{
        public:
        ImagePreprocessor (cv::Size target_size);
        cv::Mat process(cv::Mat image);
        cv::Mat process(std::string image_path);
        private:
        cv::Size m_image_size;
            cv::Mat convertToExG(cv::Mat&);
    };
}

#endif //NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H

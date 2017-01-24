//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H
#define NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H


class ImagePreprocessor{
    public:
        ImagePreprocessor (std::string images_path, cv::Size target_size);
        std::vector<cv::Mat> process();
    private:
        cv::Size m_size;
        std::string m_images_folder = "";
        cv::Mat convertToExG(cv::Mat&);
};

#endif //NEW_CROP_ROW_DETECTION_IMAGEPREPROCESSOR_H

//
// Created by noflip on 21/11/16.
//

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImagePreprocessor.h"

ImagePreprocessor::ImagePreprocessor(std::string images_path, cv::Size target_size){
    std::cout << "initd ImagePreprocessor " << std::endl;
    m_size = target_size;
    m_images_folder = images_path;
}
cv::Mat ImagePreprocessor::convertToExG(cv::Mat &image){
    cv::Mat intensity = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::Mat bgr[3];   //destination array
    cv::split(image, bgr);
    intensity = 2 * bgr[1] - bgr[2] - bgr[0];
    return intensity;
};
std::vector<cv::Mat> ImagePreprocessor::process(){
    std::vector<cv::Mat> images;
    cv::String path(m_images_folder + "*"); //select only jpg
    std::vector<cv::String> file_names;
    cv::glob(path, file_names, true); // recurse

    std::cout << "resizing images to " << m_size  << std::endl;
    for (std::string file_path : file_names)
    {
        std::cout << "loading " << file_path  << std::endl;
        cv::Mat image = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);
        // if (im.empty()) continue; //only proceed if
        cv::Mat small_image;
        cv::resize(image, small_image, m_size); // settings["image_size"]);
        cv::Mat intensity = ImagePreprocessor::convertToExG(small_image);
        images.push_back(intensity);
    }
    return images;
}


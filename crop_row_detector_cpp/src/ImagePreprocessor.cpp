//
// Created by noflip on 21/11/16.
//

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "ImagePreprocessor.h"

ImagePreprocessor::ImagePreprocessor(std::string images_path, cv::Size target_size){
    m_size = target_size;
    m_images_folder = images_path;
}
cv::Mat ImagePreprocessor::convertToExG(cv::Mat &image){
    cv::Mat intensity = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::Mat bgr[3];   //destination array
    cv::split(image, bgr);
    
    double blueMax = 0, greenMax = 0, redMax = 0;
    // Finding maximum values in all channels
    
    for( int c = 0; c < bgr[0].cols; c++) {
      for( int r = 0; r < bgr[0].rows; r++) {  
	  blueMax = (bgr[0].at<uchar>(c,r)>blueMax)?bgr[0].at<uchar>(c,r):blueMax;
	  greenMax = (bgr[1].at<uchar>(c,r)>greenMax)?bgr[1].at<uchar>(c,r):greenMax;
	  redMax = (bgr[2].at<uchar>(c,r)>redMax)?bgr[2].at<uchar>(c,r):redMax;
      }
    }
      
    double blueNorm, greenNorm, redNorm;
    double blue, green, red;
    double sumNorm, ExG;
    
    for( int c = 0; c < bgr[0].cols; c++) {
      for( int r = 0; r < bgr[0].rows; r++) {
		//normalize all pixels with max value of every channel
		blueNorm = (double) bgr[0].at<uchar>(c,r) / blueMax;
		greenNorm = (double) bgr[1].at<uchar>(c,r) / greenMax;
		redNorm = (double) bgr[2].at<uchar>(c,r) / redMax;

		sumNorm = blueNorm + greenNorm + redNorm;

		//normalize every pixel so that sum of all channels is 1
		blue = blueNorm / sumNorm;
		green = greenNorm / sumNorm;
		red = redNorm / sumNorm;

		ExG = ((2*green - blue - red) > 0) ? (2*green - blue - red)*255.0 : 0;

		intensity.at<uchar>(c,r) = (unsigned char) ExG;

      }
    }
      
    return intensity;
};
std::vector<cv::Mat> ImagePreprocessor::process(){
    std::vector<cv::Mat> images;
    cv::String path(m_images_folder + "*"); //select only jpg
    std::vector<cv::String> file_names;
    cv::glob(path, file_names, true); // recurse

    for (std::string file_path : file_names)
    {
        cv::Mat image = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);
        // if (im.empty()) continue; //only proceed if
        cv::Mat small_image;
        cv::resize(image, small_image, m_size); // settings["image_size"]);
        cv::Mat intensity = ImagePreprocessor::convertToExG(small_image);
        images.push_back(intensity);
    }
    return images;
}


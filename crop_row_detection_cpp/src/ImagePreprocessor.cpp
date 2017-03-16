//
// Created by noflip on 21/11/16.
//

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <CropRowDetector.h>
#include "ImagePreprocessor.h"

namespace crd_cpp {
    ImagePreprocessor::ImagePreprocessor(cv::Size target_size) {
        m_image_size = target_size;
    }

    cv::Mat ImagePreprocessor::convertToExG(cv::Mat &image) {

        cv::Mat intensity = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::Mat bgr[3];   //destination array
        cv::split(image, bgr);

        double blueMax, greenMax, redMax;
        double blueMin, greenMin, redMin;
        // Finding maximum values in all channels
        cv::minMaxLoc(bgr[0], &blueMin, &blueMax);
        cv::minMaxLoc(bgr[1], &greenMin, &greenMax);
        cv::minMaxLoc(bgr[2], &redMin, &redMax);

        double blueNorm, greenNorm, redNorm;
        double blue, green, red;
        double sumNorm, ExG;

        for (int c = 0; c < bgr[0].rows; c++) {
            uchar *blue_ptr = bgr[0].ptr<uchar>(c);
            uchar *green_ptr = bgr[1].ptr<uchar>(c);
            uchar *red_ptr = bgr[2].ptr<uchar>(c);
            uchar *intensity_ptr = intensity.ptr<uchar>(c);
            for (int r = 0; r < bgr[0].cols; r++, blue_ptr++, green_ptr++, red_ptr++, intensity_ptr++) {
                //normalize all pixels with max value of every channel
                blueNorm = (double) *blue_ptr / blueMax;
                greenNorm = (double) *green_ptr / greenMax;
                redNorm = (double) *red_ptr / redMax;

                sumNorm = blueNorm + greenNorm + redNorm;

                //normalize every pixel so that sum of all channels is 1
                blue = blueNorm / sumNorm;
                green = greenNorm / sumNorm;
                red = redNorm / sumNorm;

                ExG = ((2 * green - blue - red) > 0) ? (2 * green - blue - red) * 255.0 : 0;

                *intensity_ptr = (unsigned char) ExG;

            }
        }

        return intensity;
    }
    cv::Mat ImagePreprocessor::process(std::string image_path) {
        cv::Mat image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
        return process(image);
    }

    cv::Mat ImagePreprocessor::process(cv::Mat image) {
            cv::Mat resized_image;
            cv::resize(image, resized_image, m_image_size);
            cv::Mat intensity = ImagePreprocessor::convertToExG(resized_image);

            cv::Mat down_ExG_;
            down_ExG_ = cv::Mat::zeros(cv::Size(intensity.cols / 2, intensity.rows / 2), intensity.type());
            // TODO: improve ExG to become ExBrown if Green>Brown

            // Downsampling ottimo
            for (int row = 0; row < intensity.rows / 2; row++) {
                for (int column = 0; column < intensity.cols / 2; column++) {

                    int max = intensity.at<uchar>((row - 1) * 2, (column - 1) * 2);
                    if (intensity.at<uchar>((row - 1) * 2 + 1, (column - 1) * 2) > max)
                        max = intensity.at<uchar>((row - 1) * 2 + 1, (column - 1) * 2);
                    if (intensity.at<uchar>((row - 1) * 2, (column - 1) * 2 + 1) > max)
                        max = intensity.at<uchar>((row - 1) * 2, (column - 1) * 2 + 1);
                    if (intensity.at<uchar>((row - 1) * 2 + 1, (column - 1) * 2 + 1) > max)
                        max = intensity.at<uchar>((row - 1) * 2 + 1, (column - 1) * 2 + 1);

                    assert(max <= UCHAR_MAX);
                    down_ExG_.at<uchar>(row, column) = (uchar) max;
                }

            }
        return intensity;
    }
}


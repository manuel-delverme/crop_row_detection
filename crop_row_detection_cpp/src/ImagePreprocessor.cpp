//
// Created by noflip on 21/11/16.
//

#include <string>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <CropRowDetector.h>
#include "ImagePreprocessor.h"

namespace crd_cpp {
    ImagePreprocessor::ImagePreprocessor(std::string images_path, cv::Size target_size) {
        m_size = target_size;
        m_images_folder = images_path;
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
            for (int r = 0; r < bgr[0].cols; r++) {
                //normalize all pixels with max value of every channel
                blueNorm = (double) bgr[0].at<uchar>(c, r) / blueMax;
                greenNorm = (double) bgr[1].at<uchar>(c, r) / greenMax;
                redNorm = (double) bgr[2].at<uchar>(c, r) / redMax;

                sumNorm = blueNorm + greenNorm + redNorm;

                //normalize every pixel so that sum of all channels is 1
                blue = blueNorm / sumNorm;
                green = greenNorm / sumNorm;
                red = redNorm / sumNorm;

                ExG = ((2 * green - blue - red) > 0) ? (2 * green - blue - red) * 255.0 : 0;

                intensity.at<uchar>(c, r) = (unsigned char) ExG;

            }
        }

        return intensity;
    }

    std::map<period_type, std::vector<phase_type> >
    ImagePreprocessor::get_Xs(const period_type m_mind, const int n_periods, const double m_dstep) {
        std::map<period_type, std::vector<phase_type>> Xs;
        std::vector<phase_type> phases;
        period_type period = m_mind;
        for (size_t period_idx = 0; period_idx < n_periods; period_idx++, period *= m_dstep) {
            phases.clear();
            // period = std::trunc(period * 100000) / 100000;
            int half_band = (int) round(0.5 * period);

            for (int phase = -half_band; phase < half_band; phase++) {
                phases.push_back(phase);
            }
            assert(phases.size() % 2 == 0);
            Xs[period] = phases;
        }
        return Xs;
    }

    std::vector<cv::Mat> ImagePreprocessor::process() {
        std::vector<cv::Mat> images;
        cv::String path(m_images_folder + "*.jpg");
        std::vector<cv::String> file_names;
        cv::glob(path, file_names, true); // recurse

        for (std::string file_path : file_names) {
            cv::Mat image = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);
            if (image.empty()) {
                std::cout << "image: " << file_path << " was empty" << std::endl;
                continue; //only proceed if
            }
            cv::Mat intensity = ImagePreprocessor::convertToExG(image);
            cv::Mat resized_image;
            cv::resize(intensity, resized_image, m_size); // settings["image_size"]);

            images.push_back(resized_image);
        }
        return images;
    }
}

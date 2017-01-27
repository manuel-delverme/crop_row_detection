#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>
#include "ImagePreprocessor.h"

namespace crd_cpp {
#define DEBUG 0

    void display_img(cv::Mat image) {
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", image);
        cv::waitKey(0);

        // close the window
        cv::destroyWindow("Display Image");
    }

    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<old_tuple_type> &match_results) {
        cv::Mat temp_image;
        cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);
        size_t image_height = (size_t) pIntensityImg.size[0];
        size_t image_width = (size_t) pIntensityImg.size[1];
        old_tuple_type x;

        for (size_t image_row_num = 0; image_row_num < image_height; image_row_num++) {
            x = match_results.at(image_row_num);
            phase_type phase = x.first;
            period_type period = x.second;
            period_type center_of_image = (period_type) std::round(image_width / 2);
            // std::cout << image_row_num << " " << phase << "," << period << std::endl;
            period_type column = center_of_image + (period_type) phase;
            while (column < image_width) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                pPixel[2] = 255;
                column += period;
            }

            column = center_of_image + (period_type) phase - period;
            while (column >= 0) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                // std::cerr << "<drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column -= period;
            }
        }
        display_img(temp_image);
    }


    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<tuple_type> &match_results) {
        cv::Mat temp_image;
        cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);
        size_t image_height = (size_t) pIntensityImg.size[0];
        size_t image_width = (size_t) pIntensityImg.size[1];
        old_tuple_type x;

        for (size_t image_row_num = 0; image_row_num < image_height; image_row_num++) {
            x = match_results.at(image_row_num);
            phase_type phase = x.first;
            period_type period = x.second;
            period_type center_of_image = (period_type) std::round(image_width / 2);
            // std::cout << image_row_num << " " << phase << "," << period << std::endl;
            period_type column = center_of_image + (period_type) phase;
            while (column < image_width) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                pPixel[2] = 255;
                column += period;
            }

            column = center_of_image + (period_type) phase - period;
            while (column >= 0) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                // std::cerr << "<drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column -= period;
            }
        }
        display_img(temp_image);
    }

    int main_old(int argc, char **argv) {
        if (argc != 2) {
            return -1;
        }
        std::map<std::string, double> settings; // setup();
        settings["a0"] = 1.28;
        settings["b0"] = 4.48;
        settings["width"] = 400;
        settings["height"] = 300;
        // settings["d_min"] = 8;
        // settings["n_samples_per_octave"] = 70;
        // settings["n_octaves"] = 5;

        cv::Size image_size = cv::Size((uint) settings["width"], (uint) settings["height"]);
        ImagePreprocessor preprocessor(argv[1], image_size);

        CropRowDetector row_detector = CropRowDetector();
        std::map<period_type, std::vector<phase_type>> Xs;
#if !DEBUG
        Xs = preprocessor.get_Xs(row_detector.m_mind, row_detector.m_nd, row_detector.m_dstep);
#endif

        std::vector<std::vector<std::vector<energy_type>>> energy_map;

        // std::vector<std::map<old_tuple_type, double>> energy_map((size_t) image_size.height);
        std::vector<cv::Mat> images = preprocessor.process();

        for (cv::Mat &pIntensityImg : images) {

            row_detector.load(pIntensityImg);
            std::vector<energy_type> max_by_row = row_detector.template_matching(energy_map, pIntensityImg, Xs,
                                                                                       settings["a0"], settings["b0"],
                                                                                       (size_t) settings["width"]);
            std::vector<old_tuple_type> min_energy_results = row_detector.find_best_parameters(energy_map, max_by_row);

            plot_template_matching(pIntensityImg, min_energy_results);

        }
        row_detector.teardown();

        std::cout << "done" << std::endl;
        return 0;
    }
}

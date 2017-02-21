#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>
#include "crop_row_detection.h"

namespace crd_cpp {
    void display_img(cv::Mat image) {
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", image);
        cv::waitKey(0);
        cv::destroyWindow("Display Image");
    }

    void dump_template_matching(std::vector<old_tuple_type> &match_results, int image_width, std::string prefix) {
        old_tuple_type x;
        std::cout << "dumping " << match_results.size() << " results" << std::endl;

        std::ofstream csv_stream("/tmp/match_results" + prefix + ".csv");

        period_type center_crop;
        period_type left_crop;
        period_type right_crop;
        for (size_t image_row_num = 0; image_row_num < match_results.size(); image_row_num++) {
            x = match_results.at(image_row_num);
            phase_type phase = x.first;
            period_type period = x.second;

            period_type center_of_image = (period_type) std::round(image_width / 2);

            center_crop = center_of_image + (period_type) phase;
            left_crop = center_crop - period;
            right_crop = center_crop + period;

            // std::cout
            //         << image_row_num << ", "
            //         << left_crop << ", "
            //         << center_crop << ", "
            //         << right_crop << std::endl;
            // csv_stream
            //         << left_crop << ", "
            //         << center_crop << ", "
            //         << right_crop << std::endl;
            csv_stream
                    << image_row_num << " "
                    << phase << " "
                    << period << std::endl;

        }
        csv_stream.close();
    }

    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<old_tuple_type> &match_results) {
        cv::Mat temp_image;
        cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);
        size_t image_height = (size_t) pIntensityImg.size[0];
        size_t image_width = (size_t) pIntensityImg.size[1];
        old_tuple_type x;
        std::cout << "plotting " << match_results.size() << " results" << std::endl;

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
        // if (argc != 2) {
        //     return -1;
        // }
        // std::map<std::string, double> settings; // setup();
        // settings["a0"] = 1.28;
        // settings["b0"] = 4.48;
        // settings["width"] = 400;
        // settings["m_top_image_col"] = 300;
        // // settings["d_min"] = 8;
        // // settings["n_samples_per_octave"] = 70;
        // // settings["n_octaves"] = 5;

        // cv::Size image_size = cv::Size((uint) settings["width"], (uint) settings["m_top_image_col"]);
        // ImagePreprocessor preprocessor(argv[1], image_size);

        // CropRowDetector row_detector = CropRowDetector();

        // cv::Mat intensityImg = preprocessor.process("path/to/img");

        // row_detector.load(image_size);
        // row_detector.template_matching(intensityImg);
        // std::vector<old_tuple_type> min_energy_results = row_detector.find_best_parameters(row_detector.m_energy_map, row_detector.m_best_energy_by_row);

        // plot_template_matching(intensityImg, min_energy_results);

        // row_detector.teardown();

        // std::cout << "done" << std::endl;
        return 0;
    }
}

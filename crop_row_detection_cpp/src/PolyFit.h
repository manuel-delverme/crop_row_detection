//
// Created by awok on 28/07/17.
//

#ifndef CROP_ROW_DETECTION_CPP_POLYFIT_H
#define CROP_ROW_DETECTION_CPP_POLYFIT_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ctime>
#include "CropRowDetector.h"

namespace crd_cpp {
    class Polyfit {
    private:

        void plot_fitted_polys(std::string suffix);

        cv::Mat
        drawDenseOptFlow(const cv::Mat &flow, const cv::Mat &img, int step, cv::Scalar color, const cv::Mat &mask);

        void fit_poly_on_points(std::vector<crd_cpp::old_tuple_type> points, double std, double mean);

        cv::Mat calculate_flow(const cv::Mat &new_frame);

        double fit_poly_on_image(const cv::Mat &new_frame, const int max_num_iterations, const int max_useless_iterations, const double function_tolerance);

        double eval_poly_loss(const double* m_parameters, int batch_size, const bool only_central, const bool sub_pixel);

        double
        fit_central(const int max_useless_iterations, const int max_num_iterations, const double function_tolerance);
        double fit_perspective(const int max_useless_iterations, const int max_num_iterations,
                               const double function_tolerance);

        cv::Mat m_intensity_map;

        double m_parameters[5+8+1] = {
                0, 0, 0, 0, 0, // polynomial
                .01, .01, .01, .01, .01, .01, .01, .01, // perspective
                100, // period
        };

        std::vector<cv::Point2f> m_crop_row_points;

        void fit_poly_on_points();
        int m_image_center;
        int m_image_height;
        int m_image_width;
        const double lr = 1e-3;
        const double gamma = 0.9;

        const double *init_step_size(double* step_size, bool skip_lateral_rows);

    public:
        const static size_t PERSPECTIVE_OFFSET = 5;
        const static size_t PERIOD_OFFSET = 8+5;
        const static size_t NR_POLY_PARAMETERS = 8+5+1;
        cv::Mat m_drawable_image;

        static const int eval_poly(int image_row_num, int poly_idx, const double *params);
        static const double eval_poly_double(int image_row_num, int poly_idx, const double *params);
        const double eval_poly_central(int image_row_num, int poly_idx, const double *params);

        Polyfit(cv::Mat &intensity_map, cv::Mat out_img, const int max_num_iterations,
                const int max_useless_iterations, const double function_tolerance);

        double fit(cv::Mat new_frame, const int max_num_iterations, const int max_useless_iterations, const double function_tolerance);

        void add_noise();
        void test_noise(const int max_num_iterations, const int max_useless_iterations, clock_t start);
    };
}


#endif //CROP_ROW_DETECTION_CPP_POLYFIT_H

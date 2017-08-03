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

        double
        fit_central(const int max_useless_iterations, const int max_num_iterations, const double function_tolerance,
                    const int thread_idx);
        double
        fit_full(const int max_useless_iterations, const int max_num_iterations, const double function_tolerance,
        const int thread_idx);

        cv::Mat m_intensity_map;

        double m_parameters[4+6+1] = {
                0, 0, 0, 0, // polynomial
                .01, .01, .01, .01, .01, .01, // perspective
                100, // period
        };

        std::vector<cv::Point2f> m_crop_row_points;

        void fit_poly_on_points();
        int m_image_center;
        int m_image_height;
        int m_image_width;
        const double lr = 1e-1;
        const double gamma = 0.9;

        const double *init_step_size(double* step_size, bool skip_lateral_rows);
        cv::Mat m_gaussian_3_intensity_map;
        cv::Mat m_gaussian_6_intensity_map;
        cv::Mat m_gaussian_9_intensity_map;
        double eval_poly_loss(const double* m_parameters, int batch_size, const bool only_central, const bool sub_pixel);

    public:
        const static int NR_THREADS = 4;
        const static int PERSPECTIVE_OFFSET = 4;
        const static int PERIOD_OFFSET = 2*(PERSPECTIVE_OFFSET - 1) + PERSPECTIVE_OFFSET;
        const static int NR_POLY_PARAMETERS = PERIOD_OFFSET + 1;

        double m_thread_parameters[NR_THREADS][NR_POLY_PARAMETERS];
        double m_thread_losses[NR_THREADS];

        cv::Mat m_original_image;
        cv::Mat m_drawable_image;

        const int eval_poly(int image_row_num, int poly_idx, const double *params);
        const double eval_poly_double(int image_row_num, int poly_idx, const double *params);
        const double eval_poly_central(int image_row_num, int poly_idx, const double *params);

        Polyfit(cv::Mat &intensity_map,
                cv::Mat out_img,
                std::vector<old_tuple_type> ground_truth,
                const int max_num_iterations,
                const int max_useless_iterations, const double function_tolerance);

        double fit(cv::Mat new_frame, const int max_num_iterations, const int max_useless_iterations, const double function_tolerance);

        void add_noise();
        void test_noise(const int max_num_iterations, const int max_useless_iterations, clock_t start);
    };
}


#endif //CROP_ROW_DETECTION_CPP_POLYFIT_H

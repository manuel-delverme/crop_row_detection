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

        void draw_crop(const double *polynomial, const double *perspective_factors, const cv::Mat &drawImg_, double x0,
                       int image_row_num);

        void plot_crop_points(std::string suffix);

        void plot_polys(const cv::Mat &inpImg_, const double *polynomial, const double *perspective_factors,
                        double poly_period, const cv::Mat drawImg_, int poly_origin_center, double poly_origin_period);

        int eval_poly(int image_row_num, int poly_idx);

        void plot_fitted_polys(std::string suffix);

        cv::Mat
        drawDenseOptFlow(const cv::Mat &flow, const cv::Mat &img, int step, cv::Scalar color, const cv::Mat &mask);

        void fit_poly_on_points(std::vector<crd_cpp::old_tuple_type> points, double std, double mean);

        void add_noise_to_polys(double std);

        cv::Mat calculate_flow(const cv::Mat &new_frame);

        void calculate_poly_points();

        double fit_poly_on_image(const int max_num_iterations, const int max_useless_iterations);

        double eval_poly_loss(const double *poly, const double *perspect, const double period, const int margin,
                              const bool only_central, const bool sub_pixel);

        double
        fit_central(const int max_useless_iterations, const int max_num_iterations, const double function_tolerance);

        double fit_perspective(const int max_useless_iterations, const int max_num_iterations,
                               const double function_tolerance);

        cv::Mat m_intensity_map;

        /*
        double m_parameters = {
                0, 0, 0, 0, 0, // polynomial
                .01, .01, .01, .01, .01, .01, .01, .01, // perspective
                100, // period
        };
         */
        double m_polynomial[5] = {0, 0, 0, 0, 0}; // polynomial
        double m_perspective_factors[8] = {.01, .01, .01, .01, .01, .01, .01, .01}; // perspective
        double m_poly_period = 100; // period
        const size_t PERSPECTIVE_OFFSET = 5;
        const size_t PERIOD_OFFSET = 14;

        double m_best_polynomial[5] = {0, 0, 0, 0, 0};
        double m_best_perspective_factors[8] = {.01, .01, .01, .01, .01, .01, .01, .01};
        double m_best_poly_period = 100;

        std::vector<cv::Point2f> m_crop_row_points;
        cv::Mat m_spammable_img;

        void fit_poly_on_points();

        int m_image_center;
        int m_image_height;
        double m_mean;

        const double *init_step_size(double *step_size);

    public:
        cv::Mat m_image;

        static const int eval_poly(int image_row_num, int poly_idx, const double m_polynomial[5],
                                   const double m_perspective_factors[8], const double *m_poly_period);

        static const double eval_poly_double(int image_row_num, int poly_idx, const double m_polynomial[5],
                                             const double m_perspective_factors[8], const double *m_poly_period);

        Polyfit(cv::Mat &intensity_map, cv::Mat out_img, const int max_num_iterations,
                const int max_useless_iterations);

        double fit(cv::Mat new_frame, const int max_num_iterations, const int max_useless_iterations);

        void add_noise();

        void test_noise(const int max_num_iterations, const int max_useless_iterations, clock_t start);
    };
}


#endif //CROP_ROW_DETECTION_CPP_POLYFIT_H

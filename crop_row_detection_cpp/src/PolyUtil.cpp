//
// Created by awok on 30/07/17.
//

#include "PolyFit.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ctime>
#include "CropRowDetector.h"
#include <boost/thread.hpp>


template <typename T>
void print(T t)
{
    std::cout << t << " ";
}

template<typename T, typename... Args>
void print(T t, Args... args)
{
    std::cout << t << " ";
    print(args...) ;
}

namespace crd_cpp {
    class PointResidual {
    public:
        static ceres::CostFunction *Create(double x, double y, int poly_idx) {
            return (new ceres::AutoDiffCostFunction<PointResidual, Polyfit::NR_POLY_PARAMETERS, 1>(
                    new PointResidual(x, y, poly_idx)));
        }

        template<typename T>
        bool operator()(const T *params, T *residual) const {
            residual[0] = T(target_column) - eval_poly_double(row_num_, poly_idx_, params);
            return true;
        }
    public:
        template<typename T>
        const T eval_poly_double(double image_row_num, double poly_idx, const T *params) const {
            // *polynomial, const T *perspective_factors, const T *poly_period

            //TODO CHECK IF REMOVE COMMENT
            // image_row_num = 300 - image_row_num;
            T column;
            T poly_origin = params[0] + params[Polyfit::PERIOD_OFFSET] * poly_idx;
            column = (
                    // degree change
                    //  (perspective_factors[1] + poly_origin * perspective_factors[0]) * polynomial[0] * pow(T(image_row_num), 4)
                    // + (params[Polyfit::PERSPECTIVE_OFFSET][7] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET][2]) * params[3] * pow(T(image_row_num), 3)
                    + (params[Polyfit::PERSPECTIVE_OFFSET+5] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET+4]) * params[3] * pow(T(image_row_num), 3)
                    + (params[Polyfit::PERSPECTIVE_OFFSET+3] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET+2]) * params[2] * pow(T(image_row_num), 2)
                    + (params[Polyfit::PERSPECTIVE_OFFSET+1] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET+0]) * params[1] * T(image_row_num)
                    + params[0]
                    + T(poly_idx) * params[Polyfit::PERIOD_OFFSET]
            );
            return column;
        }
        PointResidual(double row_num, double col_num, int poly_idx)
                : row_num_(row_num), target_column(col_num), poly_idx_(poly_idx) {}
        double row_num_;
        double target_column;
        double poly_idx_;
    };

    /*
    void Polyfit::test_noise(const cv::Mat &intensity_map, const int max_num_iterations, const int max_useless_iterations, double min_score) {
        clock_t start;
        for (int mean = 0; mean < 100; mean += 10) {
            for (int stdev = 0; stdev < 100; stdev += 10) {
                // fit_poly_on_points(ground_truth, mean, stdev);
                plot_fitted_polys("initial fit" + std::__cxx11::to_string(mean) + "," + std::__cxx11::to_string(stdev));
                start = clock();
                assert("DISABLED" == "FIXME");
                double cost = 1;// fit_poly_on_image(intensity_map, max_num_iterations, max_useless_iterations);
                std::cout << cost << "; fit time: " << (clock() - start) / (double) (CLOCKS_PER_SEC / 1000)
                          << " ms" << std::endl;
                plot_fitted_polys("imap fit" + std::__cxx11::to_string(mean) + "," + std::__cxx11::to_string(stdev));
            }
        }
    }
    */

    const double *Polyfit::init_step_size(double* step_size, bool skip_lateral_rows) {
        double params[NR_POLY_PARAMETERS];
        const bool sub_pixel_accuracy = false;

        // degree chagne
        size_t nr_parameters = skip_lateral_rows? PERSPECTIVE_OFFSET : NR_POLY_PARAMETERS;

        for (int idx = 0; idx < nr_parameters; idx++) params[idx] = m_parameters[idx];
        // initialize with a small value scaled by the current value or 1e-33
        for (int idx = 0; idx < nr_parameters; idx++) step_size[idx] = 1e-33 + std::abs(m_parameters[idx]) * 1e-10;


        for (int idx = 0; idx < nr_parameters; idx++) {
            double fx = eval_poly_loss(m_parameters, m_image_height, skip_lateral_rows, sub_pixel_accuracy);
            double worked = 0;
            // std::cout << idx << " " << step_size[idx] << " to ";
            while (worked == 0) {
                const double h = step_size[idx];
                params[idx] += h;
                double fxh = eval_poly_loss(params, m_image_height, skip_lateral_rows, sub_pixel_accuracy);
                params[idx] -= h;
                worked = fx - fxh;
                if (worked == 0) {
                    step_size[idx] *= 2;
                }
            }
            do {
                step_size[idx] *= 0.99;
                const double h = step_size[idx];
                params[idx] += h;
                double fxh = eval_poly_loss(params, m_image_height, skip_lateral_rows, sub_pixel_accuracy);
                params[idx] -= h;
                worked = fx - fxh;
            } while (worked != 0);
            step_size[idx] /= 0.99;
            step_size[idx] /= 10;
            // std::cout << step_size[idx] << std::endl;
        }
        return step_size;
    }
    void Polyfit::fit_poly_on_points(std::vector<crd_cpp::old_tuple_type> points, double mean, double std) {
        ceres::Solver::Options m_options;
        ceres::Problem m_problem;
        ceres::Solver::Summary m_summary;

        m_options.max_num_iterations = 50;
        m_options.function_tolerance = 1e-10;
        m_options.parameter_tolerance = 1e-14;
        m_options.linear_solver_type = ceres::DENSE_QR;
        m_options.minimizer_progress_to_stdout = false;

        period_type crop_row_center, crop_row_period;
        int poly_idx;
        ceres::CostFunction *cost_function;
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(mean, std);

        for (int row_num = 0; row_num < m_image_height; row_num++) {
            crop_row_center = m_image_center + points.at((size_t) row_num).first;
            crop_row_period = points.at((size_t) row_num).second;
            for (poly_idx = -1; poly_idx < 2; poly_idx++) {
                // cost_function = PointResidual::Create(row_num, crop_row_center + poly_idx * crop_row_period, poly_idx);
                cost_function = PointResidual::Create(row_num, crop_row_center + poly_idx * crop_row_period +
                                                               distribution(generator), poly_idx);
                m_problem.AddResidualBlock(cost_function, NULL, m_parameters);
            }
        }
        ceres::Solve(m_options, &m_problem, &m_summary);
    }


    void Polyfit::fit_poly_on_points() {
        ceres::Solver::Options m_options;
        ceres::Problem m_problem;
        ceres::Solver::Summary m_summary;

        m_options.max_num_iterations = 50;
        m_options.function_tolerance = 1e-10;
        m_options.parameter_tolerance = 1e-14;
        m_options.linear_solver_type = ceres::DENSE_QR;
        m_options.minimizer_progress_to_stdout = false;

        int poly_idx;
        ceres::CostFunction *cost_function;
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(5.0, 5.1);
        for (int row_num = 0; row_num < m_image_height; row_num++) {
            for (poly_idx = -1; poly_idx < 2; poly_idx++) {
                auto p = m_crop_row_points.at((size_t) (row_num * 3 + (1 + poly_idx)));
                cost_function = PointResidual::Create(p.y + distribution(generator), p.x + distribution(generator),
                                                      poly_idx);
                m_problem.AddResidualBlock(cost_function, NULL, m_parameters);
            }
        }
        ceres::Solve(m_options, &m_problem, &m_summary);
    }

    void Polyfit::plot_fitted_polys(std::string suffix) {
        std::cout << "[plotting]: " << suffix << std::endl;
        cv::Mat img = m_drawable_image.clone();
        cv::Scalar_<double> color;

        for (int image_row_num = 0; image_row_num < m_image_height; image_row_num++) {
            for (int poly_idx = -2; poly_idx < 4; poly_idx++) {
                if(poly_idx == 0) continue;

                int column = eval_poly(image_row_num, poly_idx, m_parameters);

                if (std::abs(poly_idx) < 2)
                    color = cv::Scalar(0, 0, 128);
                else
                    color = cv::Scalar(127, 127, 127);

                cv::circle(img, cv::Point2f(column, m_image_height - image_row_num), 2, color, 1);
            }
            const int poly_idx = 0;
            int column = eval_poly(image_row_num, poly_idx, m_parameters);
            color = cv::Scalar(0, 0, 255);
            cv::circle(img, cv::Point2f(column, m_image_height - image_row_num), 2, color, 1);
        }
        // cv::imshow("RGB plot_" + suffix, img);

        // cv::imshow("plot_" + suffix, img);
        cv::imwrite(suffix + ".jpg", img);
        // cv::imwrite("plot_" + suffix + ".jpg", m_spammable_img);
        // cv::waitKey(DISPLAY_FPS);
        // cv::destroyAllWindows();
    }

    const int Polyfit::eval_poly(int image_row_num, int poly_idx, const double *params) {
        return (int) eval_poly_double(image_row_num, poly_idx, params);
    }
    const double Polyfit::eval_poly_central(int image_row_num, int poly_idx, const double *params) {
        const double x3 = pow(image_row_num, 3.0);
        const double x2 = pow(image_row_num, 2.0);
        const double x1 = pow(image_row_num, 1.0);

        double column;
        column = (
                // degree change
                // + params[4] * pow(image_row_num, 4)
                + params[3] * x3
                + params[2] * x2
                + params[1] * x1
                + params[0]
        );
        return column;
    }
    double Polyfit::fit(cv::Mat new_frame, const int max_iter, const int max_useless, double function_tolerance) {
        /*
        cv::Mat optical_flow = calculate_flow(new_frame);
        for (cv::Point2f &p: m_crop_row_points) {
            p += optical_flow.at<cv::Point2f>(p);
        }
        // plot_crop_points("pts after opt flow");

        fit_poly_on_points();
        calculate_poly_points();

        // plot_fitted_polys("fit post opt flow effect");
         */
        double cost = fit_poly_on_image(new_frame, max_iter, max_useless, function_tolerance);
        return cost;
    }

    cv::Mat Polyfit::calculate_flow(const cv::Mat &new_frame) {
        // double mask_thresh = 10.0/255.0;
        // cv::Mat min_eigen, mask, optical_flow, flow_mask;
        cv::Mat optical_flow;

        // // Optional
        // //
        // cv::cornerMinEigenVal(m_intensity_map, min_eigen, 7, 3, cv::BORDER_DEFAULT);
        // // WARNING Optimize here!
        // cv::normalize(min_eigen, min_eigen, 1, 0, cv::NORM_MINMAX);
        // cv::threshold(min_eigen, min_eigen, mask_thresh, 255, cv::THRESH_BINARY);
        // // min_eigen.convertTo(mask, CV_64F);
        // min_eigen.convertTo(mask, cv::DataType<uchar>::type);

        calcOpticalFlowFarneback(m_intensity_map, new_frame, optical_flow, 0.5, 3, 15, 5, 5, 1.2,
                                 0); // TODO: optimize params
        // cv::Mat debug_flow = drawDenseOptFlow(optical_flow, m_intensity_map, 8, cv::Scalar(0, 0, 255), flow_mask);
        // cv::imshow("optical_flow" , debug_flow);
        // cv::waitKey(100);
        return optical_flow;
    }

    /**
        * @brief Provide an image with depicted sample motion vectors of the
        input dense optical flow
        *
        * @param[in] flow The dense optical flow matrix, with one motion vector
        for each pixel (2 channels, type float)
        * @param[in] img Image to be used as background for the resulting
        optical flow image (e.g., the reference image
        *            from which the flow has been computed). It must have the
        same size as flow, with one or three
        *            channels.
        * @param[in] step Sample step, in pixel (i.e, drawDenseOptFlow() will
        draw a motion vector each step pixel)
        * @param[in] color Motion vectors color
        * @param[in] mask If not empty, it must have the same size as flow, with
        one channel and type char.
        *             The motion vectors are drawn only for non-zero pixels in
        the mask.
    */
    cv::Mat Polyfit::drawDenseOptFlow(const cv::Mat &flow, const cv::Mat &img, int step,
                                      cv::Scalar color, const cv::Mat &mask) {
        if (flow.depth() != cv::DataType<float>::type || !flow.rows || !flow.cols
            || flow.channels() != 2 ||
            img.rows != flow.rows || img.cols != flow.cols || (img.channels()
                                                               != 1 && img.channels() != 3) ||
            (!mask.empty() && (mask.depth() != cv::DataType<uchar>::type ||
                               mask.channels() != 1 ||
                               mask.rows != flow.rows || mask.cols != flow.cols)))
            throw std::invalid_argument("Unsopported  or incompatible images");

        cv::Mat flow_img(flow.rows, flow.cols, cv::DataType<cv::Vec<uchar, 3>>::type);
        cv::Mat tmp_img = img;

        if (img.channels() == 1) {
            if (img.type() != cv::DataType<uchar>::type)
                img.convertTo(tmp_img, cv::DataType<uchar>::type);
            cvtColor(tmp_img, flow_img, cv::COLOR_GRAY2BGR);
        } else {
            if (img.type() != cv::DataType<cv::Vec3b>::type)
                img.convertTo(tmp_img, cv::DataType<cv::Vec3b>::type);
            tmp_img.copyTo(flow_img);
        }
        if (!mask.empty()) {
            for (int y = 0; y < flow_img.rows; y += step) {
                const uchar *mask_p = mask.ptr<uchar>(y);
                const cv::Point2f *flow_p = flow.ptr<cv::Point2f>(y);

                for (int x = 0; x < flow_img.cols; x += step, mask_p += step,
                                                   flow_p += step) {
                    if (*mask_p) {
                        const cv::Point2f &fxy = *flow_p;
                        line(flow_img, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
                    }
                }
            }
        } else {
            for (int y = 0; y < flow_img.rows; y += step) {
                const cv::Point2f *flow_p = flow.ptr<cv::Point2f>(y);
                for (int x = 0; x < flow_img.cols; x += step, flow_p += step) {
                    const cv::Point2f &fxy = *flow_p;
                    line(flow_img, cv::Point(x, y),
                         cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
                }
            }
        }
        return flow_img;
    }
}

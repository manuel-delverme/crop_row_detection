//
// Created by awok on 28/07/17.
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

    Polyfit::Polyfit(cv::Mat &out_img, cv::Mat intensity_map, const int max_num_iterations,
                     const int max_useless_iterations, const double function_tolerance) {

        m_image_center = intensity_map.cols / 2;
        m_image_width = intensity_map.cols;
        m_image_height = intensity_map.rows;

        // blur to improve convergence
        cv::Size ksize = cv::Size(3, 1);
        double sigmaX = 0;
        print("blur\n");
        cv::GaussianBlur(intensity_map, intensity_map, ksize, sigmaX);
        //cv::medianBlur(intensity_map, intensity_map, 5);
        //m_intensity_map = intensity_map;

        m_drawable_image = out_img;

        // std::reverse(ground_truth.begin(), ground_truth.end());
        clock_t start;

        // initial guess
        // fit_poly_on_points(ground_truth, 0, 0);
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) m_parameters[idx] = 1e-33;
        m_parameters[0] = m_image_center + 1e-10;

        plot_fitted_polys("initial fit");
        start = std::clock();

        double cost = fit(intensity_map, max_num_iterations, max_useless_iterations, function_tolerance);
        std::cout << cost << "; fit time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000)
                  << " ms" << std::endl;

        plot_fitted_polys("imap fit vertical" + std::to_string(std::clock()));
        // test_noise(max_num_iterations, max_useless_iterations, start);
    }


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

    double Polyfit::fit_poly_on_image(const cv::Mat &new_frame, const int max_num_iterations, const int max_useless_iterations, const double function_tolerance) {
        m_intensity_map = new_frame;
        double central_loss = fit_central(max_useless_iterations, max_num_iterations, function_tolerance);
        // double total_loss = fit_perspective(max_useless_iterations, max_num_iterations, 1e-6);
        return central_loss;
    }

    double Polyfit::fit_central(const int max_useless_iterations, const int max_num_iterations,
                                const double function_tolerance) {
        const bool skip_lateral_rows = true;
        const bool sub_pixel_accuracy = true;

        double learning_rate[NR_POLY_PARAMETERS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        double initial_loss = eval_poly_loss(m_parameters, m_image_height, skip_lateral_rows, sub_pixel_accuracy);

        int useless_iterations = 0;
        double jac_polynomial[NR_POLY_PARAMETERS];
        double velocity[NR_POLY_PARAMETERS];

        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) velocity[idx] = 0;
        for (int idx = 0; idx < PERSPECTIVE_OFFSET; idx++) learning_rate[idx] = std::pow(lr, (double) idx);
        double total_saving = 0;

        double step_size[NR_POLY_PARAMETERS];
        init_step_size(step_size, skip_lateral_rows);

        double last_iter_loss = initial_loss;
        int batch_size = m_image_height/10;

        int iter_number;
        for (iter_number = 0; iter_number < max_num_iterations; iter_number++) {

            if (batch_size != m_image_height)
                batch_size = std::min(m_image_height, (int) ((2 * (double) useless_iterations / (double) max_useless_iterations) *
                                                  (m_image_height - m_image_height/10) + m_image_height/10));

            double fx = eval_poly_loss(m_parameters, batch_size, true, true);

            bool failed_iteration = true;
            double new_loss;
            print("--------------------------", iter_number, "-------------------------------------------------");
            std::cout << std::endl;
            for (int idx = 0; idx < PERSPECTIVE_OFFSET - 1; idx++) {
                const double h = step_size[idx];

                const double old_val = m_parameters[idx];
                m_parameters[idx] += h;
                double fxh = eval_poly_loss(m_parameters, batch_size, true, true);

                // avoid precision errors
                m_parameters[idx] = old_val;

                // poly[idx] -= h;

                jac_polynomial[idx] = ((fxh - fx) / h);

                // const int hidx = (++history_idx[idx]);
                // history_idx[idx] = hidx % 20;
                // history[idx][history_idx[idx]] = jac_polynomial[idx];

                // double new_lr = 1e-18;
                // for(int t = 0; t < 20; t++)
                // new_lr += std::pow(history[idx][t], 2);
                // const double ada = learning_rate[idx] / std::sqrt(new_lr);

                // m_polynomial[idx] += ada * jac_polynomial[idx];

                const double pre_step_loss = fx;
                const double pre_step_poly = m_parameters[idx];
                const double old_velocity = velocity[idx];
                velocity[idx] = gamma * velocity[idx] + learning_rate[idx] * (jac_polynomial[idx]);
                print(idx, "\t", m_parameters[idx], "\t-=\t", velocity[idx], "\t| ");
                m_parameters[idx] -= velocity[idx];

                double saving = last_iter_loss;
                new_loss = eval_poly_loss(m_parameters, m_image_height, true, true);

                saving -= new_loss;
                std::cout << "cost: " << new_loss << " saved " << saving;
                if (saving < -function_tolerance) {
                    print("\t[STEPPING BACK] /2", "\t", learning_rate[idx]);

                    m_parameters[idx] = pre_step_poly;
                    // learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5-idx));
                    learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5.0 - idx));
                    // velocity[idx] = old_velocity;
                    // velocity[idx] *= 0.5;
                    new_loss = eval_poly_loss(m_parameters, 300, true, true);
                } else {
                    if (std::abs(saving) > function_tolerance) { // && std::abs(velocity[idx]) > 1e-6) {
                        print("\t[SUCCESS] 1.4");
                        failed_iteration = false;
                        idx = idx - 1;
                    } else {
                        print("\t[       ] 1.4");

                    }
                    print("\t", learning_rate[idx]);
                    learning_rate[idx] *= 1.4;
                    // TODO: try to reinitialize here
                    for (int other_idx = 1; other_idx < idx; other_idx++) learning_rate[other_idx] *= 1.2;
                }
                print("\t->", learning_rate[idx], "\t");
                std::cout << std::endl;
            }
            std::cout << "cost: " << new_loss << "\t TOTAL saved " << last_iter_loss - new_loss << "\t";
            last_iter_loss = new_loss;

            if (failed_iteration) {
                useless_iterations++;
                std::cout << "FAILED: " << useless_iterations;
            } else {
                useless_iterations = 0;
            };
            std::cout << std::endl;
            if (useless_iterations > max_useless_iterations) {
                std::cout << "Max useless iteration : " << useless_iterations << std::endl;
                break;
            }
            // std::cout << std::endl;
        }
        double final_loss = eval_poly_loss(m_parameters, m_image_height, true, true);
        // std::cout << "FIRST STEP: loss: " << final_loss << " iters: " << iter_number << std::endl;
        return final_loss;
    }

    const double *Polyfit::init_step_size(double* step_size, bool skip_lateral_rows) {
        double params[NR_POLY_PARAMETERS];
        const bool sub_pixel_accuracy = false;

        // degree chagne
        size_t nr_parameters = skip_lateral_rows? PERSPECTIVE_OFFSET - 1: NR_POLY_PARAMETERS;

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
            std::cout << step_size[idx] << std::endl;
        }
        return step_size;
    }

    double Polyfit::eval_poly_loss(const double* parameters, int batch_size, const bool only_central, const bool sub_pixel) {
        int indexes[batch_size];
        int poly_idx_min;
        int poly_idx_max;

        if (only_central) {
            poly_idx_min = 0;
            poly_idx_max = 0;
        } else {
            poly_idx_min = -1;
            poly_idx_max = 1;
        }

        std::uniform_int_distribution<> dis(0, m_image_height);
        std::default_random_engine generator;

        if (batch_size == m_image_height) {
            for (int idx = 0; idx < batch_size; idx++) indexes[idx] = idx;
        } else {
            // TODO pick without replacement
            generator.seed((unsigned int) std::time(NULL));
            // auto pick = dis(generator);
            for (int idx = 0; idx < batch_size; idx++) {
                indexes[idx] = dis(generator);
            }
        }

        double cost = 0;
        double intensity;
        for (int idx = 0; idx < batch_size; idx++) {
            int row_num = indexes[idx];

            for (int poly_idx = poly_idx_min; poly_idx <= poly_idx_max; poly_idx++) {
                double column;
                if(only_central)
                    column = eval_poly_central(row_num, poly_idx, parameters);
                else{
                    column = eval_poly_double(row_num, poly_idx, parameters);
                }

                if (column < 0 || column > m_image_width || (!only_central && parameters[PERIOD_OFFSET] < 10)) {
                    cost += 1;
                } else {
                    const int left = (const int) floor(column);
                    const double f_left = ((int) m_intensity_map.at<uchar>(row_num, left));
                    if (sub_pixel) {
                        const double f_right = ((int) m_intensity_map.at<uchar>(row_num, left + 1));
                        intensity = f_left + (f_right - f_left) * (column - left);
                    } else {
                        intensity = f_left;
                    }
                    cost += -intensity / 255;
                }
            }
        }
        cost /= batch_size * (poly_idx_max - poly_idx_min + 1); // normalize horizontally
        cost *= (m_image_height / batch_size); // normalize vertically
        return cost;
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
                int column = eval_poly(image_row_num, poly_idx, m_parameters);

                if (std::abs(poly_idx) < 2)
                    color = cv::Scalar(0, 0, 255);
                else
                    color = cv::Scalar(127, 127, 127);

                cv::circle(img, cv::Point2f(column, image_row_num), 2, color, 1);
                // cv::circle(m_spammable_img, cv::Point2f(column, image_row_num), 2, color, 1);
            }
        }
        // cv::imshow("RGB plot_" + suffix, img);

        // cv::imshow("plot_" + suffix, img);
        cv::imwrite("plot_" + suffix + ".jpg", img);
        // cv::imwrite("plot_" + suffix + ".jpg", m_spammable_img);
        // cv::waitKey(DISPLAY_FPS);
        // cv::destroyAllWindows();
    }

    const int Polyfit::eval_poly(int image_row_num, int poly_idx, const double *params) {
        return (int) eval_poly_double(image_row_num, poly_idx, params);
    }

    const double Polyfit::eval_poly_double(int image_row_num, int poly_idx, const double *params) {
        double column;
        double poly_origin = params[0] + params[Polyfit::PERIOD_OFFSET] * poly_idx;
        column = (
                // degree change
                //  (perspective_factors[1] + poly_origin * perspective_factors[0]) * polynomial[0] * pow(T(image_row_num), 4)
                // + (params[Polyfit::PERSPECTIVE_OFFSET][7] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET][2]) * params[3] * pow(T(image_row_num), 3)
                + (params[Polyfit::PERSPECTIVE_OFFSET+5] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET+4]) * params[3] * pow(image_row_num, 3)
                + (params[Polyfit::PERSPECTIVE_OFFSET+3] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET+2]) * params[2] * pow(image_row_num, 2)
                + (params[Polyfit::PERSPECTIVE_OFFSET+1] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET+0]) * params[1] * pow(image_row_num, 1)
                + params[0]
                + poly_idx * params[Polyfit::PERIOD_OFFSET]
        );
        return column;
    }
    const double Polyfit::eval_poly_central(int image_row_num, int poly_idx, const double *params) {
        double column;
        column = (
                // degree change
                //  (perspective_factors[1] + poly_origin * perspective_factors[0]) * polynomial[0] * pow(T(image_row_num), 4)
                // + (params[Polyfit::PERSPECTIVE_OFFSET][7] + poly_origin * params[Polyfit::PERSPECTIVE_OFFSET][2]) * params[3] * pow(T(image_row_num), 3)
                + params[3] * pow(image_row_num, 3)
                + params[2] * pow(image_row_num, 2)
                + params[1] * pow(image_row_num, 1)
                + params[0]
                + poly_idx * params[Polyfit::PERIOD_OFFSET]
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


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
            return (new ceres::AutoDiffCostFunction<PointResidual, 1, 5, 8, 1>(new PointResidual(x, y, poly_idx)));
        }

        template<typename T>
        bool operator()(const T *poly_coeff, const T *perspective_coeff, const T *poly_period, T *residual) const {
            residual[0] = T(target_column)
                          - eval_poly_double(row_num_, poly_idx_, poly_coeff, perspective_coeff, poly_period);
            return true;
        }
    public:
        template<typename T>
        const T eval_poly_double(double image_row_num, double poly_idx, const T *polynomial,
                                 const T *perspective_factors, const T *poly_period) const {
            image_row_num = 300 - image_row_num;
            T column;
            T poly_origin = polynomial[4] + *(poly_period) * poly_idx;
            column = (
                    // degree change
                    //  (perspective_factors[1] + poly_origin * perspective_factors[0]) * polynomial[0] * pow(T(image_row_num), 4)
                    + (perspective_factors[3] + poly_origin * perspective_factors[2]) * polynomial[1] * pow(T(image_row_num), 3)
                    + (perspective_factors[5] + poly_origin * perspective_factors[4]) * polynomial[2] * pow(T(image_row_num), 2)
                    + (perspective_factors[7] + poly_origin * perspective_factors[6]) * polynomial[3] * T(image_row_num)
                    + polynomial[4]
                    + T(poly_idx) * *(poly_period)
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
                     const int max_useless_iterations) {
        m_image_center = intensity_map.cols / 2;
        m_image_height = intensity_map.rows;

        cv::Size ksize = cv::Size(3, 1);
        double sigmaX = 0;
        print("blur\n");
        cv::GaussianBlur(intensity_map, m_intensity_map, ksize, sigmaX);

        //cv::medianBlur(intensity_map, m_intensity_map, 5);
        //m_intensity_map = intensity_map;
        m_crop_row_points = std::vector<cv::Point2f>((size_t) intensity_map.rows * 3);
        m_spammable_img = out_img;
        m_image = out_img;

        // std::reverse(ground_truth.begin(), ground_truth.end());
        clock_t start;

        // fit_poly_on_points(ground_truth, 0, 0);
        for (int idx = 0; idx < 4; idx++) m_polynomial[idx] = 1e-33;
        m_polynomial[4] = m_image_center + 1e-10;

        plot_fitted_polys("initial fit");
        start = std::clock();

        double cost = fit(m_intensity_map, max_num_iterations, max_useless_iterations);

        std::cout << cost << "; fit time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms"
                  << std::endl;
        plot_fitted_polys("imap fit vertical" + std::to_string(std::clock()));
        // test_noise(max_num_iterations, max_useless_iterations, start);
    }

    void Polyfit::test_noise(const int max_num_iterations, const int max_useless_iterations, clock_t start) {
        for (int mean = 0; mean < 100; mean += 10) {
            for (int stdev = 0; stdev < 100; stdev += 10) {
                // fit_poly_on_points(ground_truth, mean, stdev);
                plot_fitted_polys("initial fit" + std::__cxx11::to_string(mean) + "," + std::__cxx11::to_string(stdev));
                start = clock();
                double cost = fit_poly_on_image(max_num_iterations, max_useless_iterations);
                std::cout << cost << "; fit time: " << (clock() - start) / (double) (CLOCKS_PER_SEC / 1000)
                          << " ms" << std::endl;
                plot_fitted_polys("imap fit" + std::__cxx11::to_string(mean) + "," + std::__cxx11::to_string(stdev));
            }
        }
    }

    double Polyfit::fit_poly_on_image(const int max_num_iterations, const int max_useless_iterations) {
        // double first_loss = fit_central(max_useless_iterations, max_num_iterations, 5e-4);
        // plot_fitted_polys("imap fit first step");
        double central_loss = fit_central(max_useless_iterations, max_num_iterations, 1e-6);
        // double total_loss = fit_perspective(max_useless_iterations, max_num_iterations, 1e-6);
        // std::cout << "FIRST STEP: loss: " << first_loss << std::endl;
        return central_loss;
    }

    double Polyfit::fit_perspective(const int max_useless_iterations, const int max_num_iterations,
                                    const double function_tolerance) {
        const double lr = 1e-3;
        const double gamma = 0.9;

        double poly[5];

        double learning_rate[5] = {
                0,
                std::pow(lr, 4.0),
                std::pow(lr, 3.0),
                std::pow(lr, 2.0),
                std::pow(lr, 1.0),
        };
        double initial_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, false, true);

        // degree change
        learning_rate[0] = 0;

        int useless_iterations = 0;
        double jac_polynomial[5];
        double velocity[5];
        double step_size[5];
        int downs[5];
        int ups[5];
        for (int idx = 1; idx < 5; idx++) velocity[idx] = 0;
        for (int idx = 1; idx < 5; idx++) step_size[idx] = 1e-33 + std::abs(m_polynomial[idx]) * 1e-10;
        for (int idx = 1; idx < 5; idx++) poly[idx] = m_polynomial[idx];

        for (int idx = 1; idx < 5; idx++) {
            double fx = eval_poly_loss(poly, m_perspective_factors, m_poly_period, 300, true, false);
            double worked = 0;
            while (worked == 0) {
                const double h = step_size[idx];
                poly[idx] += h;
                double fxh = eval_poly_loss(poly, m_perspective_factors, m_poly_period, 300, true, false);
                poly[idx] -= h;
                worked = fx - fxh;
                if (worked == 0) step_size[idx] *= 2;
            }
            do {
                step_size[idx] *= 0.99;

                const double h = step_size[idx];
                poly[idx] += h;
                double fxh = eval_poly_loss(poly, m_perspective_factors, m_poly_period, 300, true, false);
                poly[idx] -= h;
                worked = fx - fxh;
            } while (worked != 0);
            step_size[idx] /= 0.99;
            step_size[idx] /= 10;
            std::cout << step_size[idx] << std::endl;
        }

        double last_iter_loss = initial_loss;
        int batch_size = 30;

        int iter_number;
        for (iter_number = 0; iter_number < max_num_iterations; iter_number++) {
            if (batch_size != 300)
                batch_size = std::min(300, (int) ((2 * (double) useless_iterations / (double) max_useless_iterations) *
                                                  (300.0 - 30.0) + 30));

            double fx = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, batch_size, true, true);

            bool failed_iteration = true;
            double new_loss;
            print("--------------------------", iter_number, "-------------------------------------------------");
            std::cout << std::endl;
            for (int idx = 4; idx > 0; idx--) {
                const double h = step_size[idx];

                const double old_val = m_polynomial[idx]; //poly[idx] += h;
                m_polynomial[idx] += h;
                double fxh = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, batch_size, true, true);

                // avoid precision errors
                m_polynomial[idx] = old_val;

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
                const double pre_step_poly = m_polynomial[idx];
                const double old_velocity = velocity[idx];
                velocity[idx] = gamma * velocity[idx] + learning_rate[idx] * (jac_polynomial[idx]);
                print(idx, "\t", m_polynomial[idx], "\t-=\t", velocity[idx], "\t| ");
                m_polynomial[idx] -= velocity[idx];

                double saving = last_iter_loss;
                new_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, true, true);

                saving -= new_loss;
                std::cout << "cost: " << new_loss << " saved " << saving;
                if (saving < -function_tolerance) {
                    print("\t[STEPPING BACK] /2");
                    print("\t", learning_rate[idx]);
                    m_polynomial[idx] = pre_step_poly;
                    // learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5-idx));
                    learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5.0 - idx));
                    ups[idx] = 0;
                    downs[idx] += 1;
                    // velocity[idx] = old_velocity;
                    // velocity[idx] *= 0.5;
                    new_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, true, true);
                } else {
                    if (std::abs(saving) > function_tolerance) { // && std::abs(velocity[idx]) > 1e-6) {
                        print("\t[SUCCESS] 1.4");
                        failed_iteration = false;
                    } else {
                        print("\t[       ] 1.4");

                    }
                    print("\t", learning_rate[idx]);
                    learning_rate[idx] *= 1.4;
                    ups[idx] += 1;
                    downs[idx] = 0;
                    for (int other_idx = 1; other_idx < idx; other_idx++) learning_rate[other_idx] *= 1.2;
                }
                print("\t->", learning_rate[idx], "\t");
                print("\t^", ups[idx], "\tv", downs[idx]);
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
        double final_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, true, true);
        // std::cout << "FIRST STEP: loss: " << final_loss << " iters: " << iter_number << std::endl;
        return final_loss;
    }

    double Polyfit::fit_central(const int max_useless_iterations, const int max_num_iterations,
                                const double function_tolerance) {
        const double lr = 1e-3;
        const double gamma = 0.9;

        double poly[5];

        double learning_rate[5] = {
                0,
                std::pow(lr, 4.0),
                std::pow(lr, 3.0),
                std::pow(lr, 2.0),
                std::pow(lr, 1.0),
        };
        double initial_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, true, true);

        // degree change
        learning_rate[0] = 0;

        int useless_iterations = 0;
        double jac_polynomial[5];
        double velocity[5];
        double history[5][20];
        int history_idx[5];
        double parameter_saving[5];
        int downs[5];
        int ups[5];
        for (int idx = 1; idx < 5; idx++) velocity[idx] = 0;
        for (int idx = 1; idx < 5; idx++) history_idx[idx] = -1;
        for (int idx = 1; idx < 5; idx++) downs[idx] = 0;
        for (int idx = 1; idx < 5; idx++) ups[idx] = 0;
        for (int idx = 1; idx < 5; idx++) parameter_saving[idx] = function_tolerance;
        for (int idx = 1; idx < 5; idx++) for (int t = 0; t < 20; t++) history[idx][t] = 0;
        double total_saving = 0;

        double step_size[5];
        init_step_size(step_size);

        double last_iter_loss = initial_loss;
        int batch_size = 30;

        int iter_number;
        for (iter_number = 0; iter_number < max_num_iterations; iter_number++) {
            if (batch_size != 300)
                batch_size = std::min(300, (int) ((2 * (double) useless_iterations / (double) max_useless_iterations) *
                                                  (300.0 - 30.0) + 30));

            double fx = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, batch_size, true, true);

            bool failed_iteration = true;
            double new_loss;
            print("--------------------------", iter_number, "-------------------------------------------------");
            std::cout << std::endl;
            for (int idx = 4; idx > 0; idx--) {
                const double h = step_size[idx];

                const double old_val = m_polynomial[idx]; //poly[idx] += h;
                m_polynomial[idx] += h;
                double fxh = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, batch_size, true, true);

                // avoid precision errors
                m_polynomial[idx] = old_val;

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
                const double pre_step_poly = m_polynomial[idx];
                const double old_velocity = velocity[idx];
                velocity[idx] = gamma * velocity[idx] + learning_rate[idx] * (jac_polynomial[idx]);
                print(idx, "\t", m_polynomial[idx], "\t-=\t", velocity[idx], "\t| ");
                m_polynomial[idx] -= velocity[idx];

                double saving = last_iter_loss;
                new_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, true, true);

                saving -= new_loss;
                std::cout << "cost: " << new_loss << " saved " << saving;
                if (saving < -function_tolerance) {
                    print("\t[STEPPING BACK] /2");
                    print("\t", learning_rate[idx]);
                    m_polynomial[idx] = pre_step_poly;
                    // learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5-idx));
                    learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5.0 - idx));
                    ups[idx] = 0;
                    downs[idx] += 1;
                    // velocity[idx] = old_velocity;
                    // velocity[idx] *= 0.5;
                    new_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, true, true);
                } else {
                    if (std::abs(saving) > function_tolerance) { // && std::abs(velocity[idx]) > 1e-6) {
                        print("\t[SUCCESS] 1.4");
                        failed_iteration = false;
                        idx = idx + 1;
                    } else {
                        print("\t[       ] 1.4");

                    }
                    print("\t", learning_rate[idx]);
                    learning_rate[idx] *= 1.4;
                    ups[idx] += 1;
                    downs[idx] = 0;
                    for (int other_idx = 1; other_idx < idx; other_idx++) learning_rate[other_idx] *= 1.2;
                }
                print("\t->", learning_rate[idx], "\t");
                print("\t^", ups[idx], "\tv", downs[idx]);
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
        double final_loss = eval_poly_loss(m_polynomial, m_perspective_factors, m_poly_period, 300, true, true);
        // std::cout << "FIRST STEP: loss: " << final_loss << " iters: " << iter_number << std::endl;
        return final_loss;
    }

    const double *Polyfit::init_step_size(double step_size[5]) {
        double poly[5];

        for (int idx = 1; idx < 5; idx++) poly[idx] = m_polynomial[idx];
        for (int idx = 1; idx < 5; idx++) step_size[idx] = 1e-33 + std::abs(m_polynomial[idx]) * 1e-10;

        for (int idx = 1; idx < 5; idx++) {
            double fx = eval_poly_loss(poly, m_perspective_factors, m_poly_period, 300, true, false);
            double worked = 0;
            // std::cout << idx << " " << step_size[idx] << " to ";
            while (worked == 0) {
                const double h = step_size[idx];
                poly[idx] += h;
                double fxh = eval_poly_loss(poly, m_perspective_factors, m_poly_period, 300, true, false);
                poly[idx] -= h;
                worked = fx - fxh;
                if (worked == 0) {
                    step_size[idx] *= 2;
                }
            }
            do {
                step_size[idx] *= 0.99;
                const double h = step_size[idx];
                poly[idx] += h;
                double fxh = eval_poly_loss(poly, m_perspective_factors, m_poly_period, 300, true, false);
                poly[idx] -= h;
                worked = fx - fxh;
            } while (worked != 0);
            step_size[idx] /= 0.99;
            step_size[idx] /= 10;
            std::cout << step_size[idx] << std::endl;
        }
        return step_size;
    }

    double Polyfit::eval_poly_loss(const double *poly, const double *perspect, const double period, int batch_size,
                                   const bool only_central, const bool sub_pixel) {
        int poly_idx_min = -1;
        int poly_idx_max = 1;
        if (only_central) {
            // consider only center row
            poly_idx_min = 0;
            poly_idx_max = 0;
        }
        int indexes[batch_size];

        std::uniform_int_distribution<> dis(0, 300);
        std::default_random_engine generator;
        generator.seed((unsigned int) std::time(NULL));

        if (batch_size == 300) {
            for (int idx = 0; idx < batch_size; idx++)
                indexes[idx] = idx;
        } else {
            auto pick = dis(generator);
            for (int idx = 0; idx < batch_size; idx++) {
                indexes[idx] = dis(generator);
            }
        }


        double cost = 0;
        double intensity;
        for (int idx = 0; idx < batch_size; idx++) {
            int row_num = indexes[idx];

            for (int poly_idx = poly_idx_min; poly_idx <= poly_idx_max; poly_idx++) {
                const double column = eval_poly_double(row_num, poly_idx, poly, perspect, &period);
                if (column < 0 || column > 398 || period < 10) {
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
        cost /= batch_size * (poly_idx_max - poly_idx_min + 1);
        cost *= (300 / batch_size);
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
                m_problem.AddResidualBlock(cost_function, NULL, m_polynomial, m_perspective_factors, &m_poly_period);
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
                m_problem.AddResidualBlock(cost_function, NULL, m_polynomial, m_perspective_factors, &m_poly_period);
            }
        }
        ceres::Solve(m_options, &m_problem, &m_summary);
    }

    void Polyfit::plot_fitted_polys(std::string suffix) {
        std::cout << "[plotting]: " << suffix << std::endl;
        cv::Mat img = m_image.clone();
        cv::Scalar_<double> color;

        for (int image_row_num = 0; image_row_num < m_image_height; image_row_num++) {
            for (int poly_idx = -2; poly_idx < 4; poly_idx++) {
                int column = eval_poly(image_row_num, poly_idx);

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

    const int Polyfit::eval_poly(int image_row_num, int poly_idx, const double polynomial[5],
                                 const double perspective_factors[8], const double *poly_period) {
        return (int) eval_poly_double(image_row_num, poly_idx, polynomial, perspective_factors, poly_period);
    }

    const double Polyfit::eval_poly_double(int image_row_num, int poly_idx, const double *polynomial,
                                           const double *perspective_factors, const double *poly_period) {
        double column;
        double poly_origin = polynomial[4] + *(poly_period) * poly_idx;
        column = (
                // degree change
                // // (perspective_factors[1] + poly_origin * perspective_factors[0]) * polynomial[0] * pow(image_row_num, 4)
                +(perspective_factors[3] + poly_origin * perspective_factors[2]) * polynomial[1] * pow(image_row_num, 3)
                +
                (perspective_factors[5] + poly_origin * perspective_factors[4]) * polynomial[2] * pow(image_row_num, 2)
                + (perspective_factors[7] + poly_origin * perspective_factors[6]) * polynomial[3] * image_row_num
                + polynomial[4]
                + poly_idx * *(poly_period)
        );
        return column;
    }

    int Polyfit::eval_poly(int image_row_num, int poly_idx) {
        return eval_poly(image_row_num, poly_idx, m_polynomial, m_perspective_factors, &m_poly_period);
    }

    double Polyfit::fit(cv::Mat new_frame, const int max_iter, const int max_useless) {
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
        m_intensity_map = new_frame;
        double cost = fit_poly_on_image(max_iter, max_useless);
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
    };
}


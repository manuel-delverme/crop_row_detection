//
// Created by awok on 28/07/17.
//

#include "PolyFit.h"
#include <iomanip>
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
    class CropRowCostFunctor {
    public:
        bool operator()(const double *poly_params, double *residuals) const {
            const bool only_central = false;
            residuals[0] = Polyfit::static_eval_poly_loss(poly_params, row_num_, only_central, true, image_);
            return true;
        }

        static ceres::CostFunction *Create(const cv::Mat image, const int row_num) {
            return new ceres::NumericDiffCostFunction<CropRowCostFunctor, ceres::CENTRAL, 1, Polyfit::NR_POLY_PARAMETERS>(
                    new CropRowCostFunctor(image, row_num));
        }

    private:
        CropRowCostFunctor(const cv::Mat &image, const int row_num) :
                image_(image), row_num_(row_num) {}

        const cv::Mat image_;
        const int row_num_;
    };

    Polyfit::Polyfit(cv::Mat &out_img, cv::Mat intensity_map, std::vector<old_tuple_type> ground_truth, const int max_num_iterations,
                     const int max_useless_iterations, const double function_tolerance) {

        m_image_center = intensity_map.cols / 2;
        m_image_width = intensity_map.cols;
        m_image_height = intensity_map.rows;

        // blur to improve convergence
        cv::Size ksize = cv::Size(3, 1);
        cv::GaussianBlur(intensity_map, m_gaussian_3_intensity_map, ksize, 0);
        ksize = cv::Size(5, 1);
        cv::GaussianBlur(intensity_map, m_gaussian_6_intensity_map, ksize, 0);
        ksize = cv::Size(9, 1);
        cv::GaussianBlur(intensity_map, m_gaussian_9_intensity_map, ksize, 0);
        //cv::medianBlur(intensity_map, intensity_map, 5);
        //m_intensity_map = intensity_map;

        m_drawable_image = out_img;
        m_original_image = out_img;

        std::reverse(ground_truth.begin(), ground_truth.end());
        clock_t start;

        // initial guess
        if(ground_truth.empty()){
            // for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) m_parameters[idx] = 0.0000125;
            // m_parameters[0] = 166.538;
            m_parameters[1] = -0.201784;
            m_parameters[2] = -0.00103746;
            m_parameters[3] = 1.96107e-06;
            m_parameters[4] = 0.00621423;
            m_parameters[5] = -2.16231;
            m_parameters[6] = 0.00720419;
            m_parameters[7] = 1.44456;
            m_parameters[8] = 0.00666877;
            m_parameters[9] = 2.52987;

            double sum_column[m_image_width];
            for (int idx = 0; idx < m_image_width; idx++) sum_column[idx] = 0;
            for(int r=0; r < intensity_map.rows; r++)
                for(int c=0; c < intensity_map.cols; c++)
                    sum_column[c] += intensity_map.at<uchar>(r, c);


            m_parameters[PERIOD_OFFSET] = m_image_width / 3;
            // m_parameters[0] = m_image_center + 1e-10;
            m_parameters[0] = 0;
            for (int idx = 0; idx < m_image_width; idx++){
                if(sum_column[idx] > sum_column[(int)m_parameters[0]]){
                    m_parameters[0] = idx;
                }
            }
            // gets stuck on step sizing
            m_parameters[0] += 1e-20;
        } else {
            fit_poly_on_points(ground_truth, 0, 0);
        }
        print("\ninitial guess:\n");
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) print(idx, m_parameters[idx], "\n");
        plot_fitted_polys("initial fit");
        start = std::clock();

        double cost = fit(intensity_map, max_num_iterations, max_useless_iterations, function_tolerance);
        std::cout << cost << "; fit time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000)
                  << " ms" << std::endl;

        plot_fitted_polys("imap fit vertical" + std::to_string(std::time(NULL)));
        // test_noise(max_num_iterations, max_useless_iterations, start);
    }

    double Polyfit::fit_poly_on_image(const cv::Mat &new_frame, const int max_num_iterations, const int max_useless_iterations, const double function_tolerance) {
        m_intensity_map = new_frame;
        print("\ninitial guess:\n");
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) print(idx, m_parameters[idx], "\n");
        double initial_loss = eval_poly_loss(m_parameters, m_image_height, true, true);
        plot_fitted_polys("fpoi");

        double initial_params[NR_POLY_PARAMETERS];
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) initial_params[idx] = m_parameters[idx];

        double central_loss;
        double best_loss = fit_full(max_useless_iterations, max_num_iterations, function_tolerance, 0);

        double best_param[NR_POLY_PARAMETERS];
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) best_param[idx] = m_parameters[idx];
        plot_fitted_polys("thread-1");

        /*
        boost::thread thread1(&Polyfit::fit_central, this, max_useless_iterations, max_num_iterations, function_tolerance, 0);
        boost::thread thread2(&Polyfit::fit_central, this, max_useless_iterations, max_num_iterations, function_tolerance, 1);
        boost::thread thread3(&Polyfit::fit_central, this, max_useless_iterations, max_num_iterations, function_tolerance, 2);
        boost::thread thread4(&Polyfit::fit_central, this, max_useless_iterations, max_num_iterations, function_tolerance, 3);

        thread1.join();
        thread2.join();
        thread3.join();
        thread4.join();

        for (int thread_idx = 0; thread_idx < NR_THREADS; thread_idx++){
            print("thread", thread_idx, "loss:", m_thread_losses[thread_idx], "\n");
            for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) m_parameters[idx] = m_thread_parameters[thread_idx][idx];
            plot_fitted_polys("thread" + std::to_string(thread_idx));

            if(m_thread_losses[thread_idx] < best_loss){
                for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) best_param[idx] = m_thread_parameters[thread_idx][idx];
                best_loss = m_thread_losses[thread_idx];
            }
        }
         */

        /*
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) m_parameters[idx] = best_param[idx];
         */
        return -1;
    }

    double Polyfit::fit_full(const int max_useless_iterations, const int max_num_iterations,
                                const double function_tolerance, const int thread_idx) {
        plot_fitted_polys("function call");
        bool skip_lateral_rows = false;
        const bool sub_pixel_accuracy = true;
        const double print_stuff = true;


        double success[NR_POLY_PARAMETERS];
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) success[idx] = -1;


        double learning_rate[NR_POLY_PARAMETERS];
        //TODO: find better parameters
        learning_rate[0] = lr;
        learning_rate[1] = learning_rate[0] * lr;
        learning_rate[2] = learning_rate[1] * lr;
        learning_rate[3] = learning_rate[2] * lr;
        // learning_rate[4] = 0.001;
        learning_rate[PERSPECTIVE_OFFSET+0] = lr;
        learning_rate[PERSPECTIVE_OFFSET+1] = lr;
        learning_rate[PERSPECTIVE_OFFSET+2] = lr * learning_rate[PERSPECTIVE_OFFSET + 1];
        learning_rate[PERSPECTIVE_OFFSET+3] = lr * learning_rate[PERSPECTIVE_OFFSET + 1];
        learning_rate[PERSPECTIVE_OFFSET+4] = lr * learning_rate[PERSPECTIVE_OFFSET + 3];
        learning_rate[PERSPECTIVE_OFFSET+5] = lr * learning_rate[PERSPECTIVE_OFFSET + 3];

        learning_rate[PERIOD_OFFSET] = lr;

        double locked[NR_POLY_PARAMETERS];
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) locked[idx] = true;
        locked[0] = false;
        locked[1] = false;
        locked[2] = false;
        locked[3] = false;
        if(locked[PERSPECTIVE_OFFSET]){
            skip_lateral_rows = true;
        } else {
            skip_lateral_rows = false;
        }

        // double learning_rate_floor[NR_POLY_PARAMETERS];
        double trust_region[NR_POLY_PARAMETERS];
        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) trust_region[idx] = learning_rate[idx];

        double initial_loss = eval_poly_loss(m_parameters, m_image_height, skip_lateral_rows, sub_pixel_accuracy);

        int useless_iterations = 0;
        double jac_polynomial[NR_POLY_PARAMETERS];
        double velocity[NR_POLY_PARAMETERS];

        for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) velocity[idx] = 0;

        double step_size[NR_POLY_PARAMETERS];
        init_step_size(step_size, skip_lateral_rows);

        double last_iter_loss = initial_loss;
        int batch_size = m_image_height;

        int iter_number;
        for (iter_number = 0; iter_number < max_num_iterations; iter_number++) {

            if(batch_size != m_image_height)
                batch_size = std::min(m_image_height, (int) ((2 * (double) useless_iterations / (double) max_useless_iterations) *
                                                             (m_image_height - m_image_height/10) + m_image_height/10));

            double fx = eval_poly_loss(m_parameters, batch_size, skip_lateral_rows, sub_pixel_accuracy);

            bool failed_iteration = true;
            double new_loss;
            if(print_stuff) print("--------------------------", iter_number, "-------------------------------------------------");
            if(print_stuff) std::cout << std::endl;
            int next_idx;
            for (int idx = 0; idx < NR_POLY_PARAMETERS; idx++) {
                if(locked[idx]) continue;
                if(locked[PERSPECTIVE_OFFSET]){
                    skip_lateral_rows = true;
                } else {
                    skip_lateral_rows = false;
                }

                const double h = step_size[idx];

                const double old_val = m_parameters[idx];
                m_parameters[idx] += h;
                // plot_fitted_polys("post-step");
                double fxh = eval_poly_loss(m_parameters, batch_size, skip_lateral_rows, sub_pixel_accuracy);

                // avoid precision errors
                m_parameters[idx] = old_val;
                // plot_fitted_polys("back-to-pre-step");

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

                const double pre_step_poly = m_parameters[idx];
                const double old_velocity = velocity[idx];
                velocity[idx] = gamma * velocity[idx] + /*learning_rate[idx] **/ (jac_polynomial[idx]);
                velocity[idx] = std::min(trust_region[idx], std::max(velocity[idx], -trust_region[idx]));
                if(print_stuff){
                    std::string name;
                    if(idx < PERSPECTIVE_OFFSET){
                       name = "poly" + std::to_string(idx);
                    } else if(idx == PERIOD_OFFSET) {
                        name = "period";
                    } else {
                        name = "perspective" + std::to_string(idx - PERSPECTIVE_OFFSET);
                    }
                    print(name, "\t", m_parameters[idx], "\t-=\t", velocity[idx], "\t| ");
                }
                m_parameters[idx] -= velocity[idx];

                double saving = last_iter_loss;

                m_drawable_image = m_original_image.clone();
                new_loss = eval_poly_loss(m_parameters, m_image_height, skip_lateral_rows, sub_pixel_accuracy);
                plot_fitted_polys("fittings/velocity-update" + std::to_string(iter_number) + "idx" + std::to_string(idx));

                saving -= new_loss;
                if(print_stuff) std::cout << "cost: " << new_loss << " saved " << saving;
                next_idx = idx;
                if (saving < -function_tolerance || jac_polynomial[idx] == 0.0) {
                    if(print_stuff) print("\t[STEPPING BACK] /2", "\t", trust_region[idx]);

                    m_parameters[idx] = pre_step_poly;
                    // learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5-idx));
                    // learning_rate[idx] = std::max(learning_rate[idx] * 0.5, std::pow(lr, 5.0 - idx));
                    // learning_rate[idx] *= 0.5;
                    trust_region[idx] *= 0.5;
                    /*
                    //TODO cache floor values to save calculations
                    if(learning_rate[idx] < learning_rate_floor[idx]){
                        learning_rate[idx] = learning_rate_floor[idx];
                    } else {
                        failed_iteration = false;
                    }
                     */
                    // velocity[idx] = old_velocity;
                    // velocity[idx] *= 0.5;
                    new_loss = eval_poly_loss(m_parameters, m_image_height, skip_lateral_rows, sub_pixel_accuracy);
                } else {
                    if (std::abs(saving) > function_tolerance) { // && std::abs(velocity[idx]) > 1e-6) {
                        failed_iteration = false;
                        if(print_stuff) print("\t[SUCCESS] 1.4");
                        // next_idx -= 1;
                        if(success[idx] < 0){
                            success[idx] = learning_rate[idx];
                        }
                    } else {
                        // print("\t[       ] 1.4");
                    }
                    if(print_stuff) print("\t", trust_region[idx]);
                    // learning_rate[idx] *= 1.4;
                    trust_region[idx] *= 1.4;
                    // TODO: try to reinitialize here
                    for (int other_idx = 1; other_idx < idx; other_idx++) learning_rate[other_idx] *= 1.2;
                }
                if(print_stuff) print("\t->", trust_region[idx], "\t");
                idx = next_idx;
                if(print_stuff) std::cout << std::endl;
            }
            if(print_stuff) std::cout << "cost: " << new_loss << "\t TOTAL saved " << last_iter_loss - new_loss << "\t";
            /*
            if(print_stuff)
                if(last_iter_loss - new_loss > function_tolerance * 1000 || iter_number % 100 == 0)
                    plot_fitted_polys("fittings/" + std::to_string(iter_number));
            */
            last_iter_loss = new_loss;

            if (failed_iteration) {
                useless_iterations++;
                if(print_stuff) std::cout << "FAILED: " << useless_iterations;
            } else {
                useless_iterations = 0;
            };
            std::cout << std::endl;
            if (useless_iterations > max_useless_iterations) {
                std::cout << "Max useless iteration : " << useless_iterations << std::endl;
                break;
            }
            if(print_stuff) std::cout << std::endl;
        }
        double final_loss = eval_poly_loss(m_parameters, m_image_height, skip_lateral_rows, sub_pixel_accuracy);
        m_thread_losses[thread_idx] = final_loss;
        return final_loss;
    }
    double Polyfit::static_eval_poly_loss(const double* parameters, int row_num, const bool only_central,
                                   const bool sub_pixel, cv::Mat m_intensity_map) {
        const int batch_size = 300;
        int poly_idx_min = -1;
        int poly_idx_max = 1;
        double cost = 0;
        double intensity;
        double column_positions[poly_idx_max - poly_idx_min];
        int m_image_width = m_intensity_map.cols;
        int m_image_height = m_intensity_map.rows;

        for (int poly_idx = poly_idx_min; poly_idx <= poly_idx_max; poly_idx++) {
            double column;
            column = eval_poly_double(row_num, poly_idx, parameters);

            // ------------------------------------------ NAN CHECK
            if(poly_idx != poly_idx_min){
                const double column_m1 = column_positions[poly_idx - poly_idx_min - 1];
                if(std::abs(column_m1 - column) < 10)
                    cost += 10;
            }
            if (column < 0 || column > m_image_width || column != column || (parameters[PERIOD_OFFSET] < m_image_width/5 && !only_central)) {
                cost += 10;
            } else {
                const int left = (const int) floor(column);
                const double f_left = ((int) m_intensity_map.at<uchar>(m_image_height - row_num, left));
                if (sub_pixel) {
                    const double f_right = ((int) m_intensity_map.at<uchar>(m_image_height - row_num, left + 1));
                    intensity = f_left + (f_right - f_left) * (column - left);
                } else {
                    intensity = f_left;
                }
                cost += -intensity / 255;
            }
        }
        cost /= batch_size * (poly_idx_max - poly_idx_min + 1); // normalize horizontally
        cost *= (m_image_height / batch_size); // normalize vertically
        return cost;

    }
    double Polyfit::eval_poly_loss(const double* parameters, int batch_size, const bool only_central,
                                   const bool sub_pixel) {
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
            generator.seed((unsigned int) std::time(NULL) * (unsigned int) (m_parameters[0]*100));
            // auto pick = dis(generator);
            for (int idx = 0; idx < batch_size; idx++) {
                indexes[idx] = dis(generator); // TODO: sort to avoid cache misses
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

                // ------------------------------------------ NAN CHECK
                if (column < 0 || column > m_image_width ) {
                    cost += 10;
                } else if (column != column) {
                    print("IS NAN\n");
                    cost += 10;
                } else if (parameters[PERIOD_OFFSET] < m_image_width/5 && !only_central) {
                    print("CROP ROW TOO NEAR\n");
                    cost += 10;
                } else {
                    const int left = (const int) floor(column);
                    const double f_left = ((int) m_intensity_map.at<uchar>(cv::Point2f(left, m_image_height - row_num)));
                    auto color = cv::Scalar(255, 0, 0);
                    cv::circle(m_drawable_image, cv::Point2f(left, m_image_height - row_num), 2, color, 1);

                    if (sub_pixel) {
                        const double f_right = (int) m_intensity_map.at<uchar>(cv::Point2f(left + 1, m_image_height - row_num));
                        intensity = f_left + (f_right - f_left) * (column - left);
                        cv::circle(m_drawable_image, cv::Point2f(left, m_image_height - row_num), 2, color, 1);
                    } else {
                        intensity = f_left;
                    }
                    cost += -intensity / 255;
                }
            }
        }
        /*
            const int poly_idx = 0;
            int column = eval_poly(image_row_num, poly_idx, m_parameters);
            color = cv::Scalar(0, 0, 255);
            cv::circle(img, cv::Point2f(column, m_image_height - image_row_num), 2, color, 1);
        }
         * */
        cost /= batch_size * (poly_idx_max - poly_idx_min + 1); // normalize horizontally
        cost *= (m_image_height / batch_size); // normalize vertically
        return cost;
    }

}


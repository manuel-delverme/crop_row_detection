//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
#include "CropRowDetector.h"

CropRowDetector::CropRowDetector (int a) {
    std::cout << "initd" << a << std::endl;
}

cv::Mat CropRowDetector::detect(cv::Mat& intensity, cv::Mat& templ){
    // Mat templ; // from settings
    cv::Mat result;
    int result_rows = intensity.rows - templ.rows;
    int result_cols = intensity.cols - templ.cols;
    result.create(result_rows, result_cols, CV_8UC1);

    cv::Mat average_intensity;
    average_intensity.create(1, intensity .cols, CV_64F);

    cv::Mat plot_image;
    plot_image.create(1, intensity.cols, CV_64F);

    cv::Ptr<cv::plot::Plot2d> plot;
    cv::Mat plot_result;
    for (int i=intensity.rows - 1; i >= intensity.rows / 2 ; i--) {
        // std::cout << "displaying row: " << i << std::endl;
        intensity.row(i).convertTo(plot_image, CV_64F);
        /*
           plot = cv::plot::createPlot2d(plot_image);
           plot->render(plot_result);
           cv::imshow("row intensity", plot_result);
           cv::waitKey(1);
           */
        // std::cout << "going to sum " << cv::Size(average_intensity);
        // std::cout << " and " << cv::Size(plot_image) << std::endl;
        average_intensity += (plot_image / intensity.rows);
    }
    // average_intensity =/ intensity.rows;
    std::cout << "displaying avg" << std::endl;
    plot = cv::plot::createPlot2d(average_intensity);
    plot->render(plot_result);
    cv::imshow("row intensity", plot_result);
    cv::waitKey(0);
    // cv::matchTemplate(intensity, templ, result, CV_TM_CCORR);
    // normalize?
    return result;
}

/*
 * returns the best pair x for each row
 * */
std::vector<std::pair<int, int>> CropRowDetector::template_matching(
        cv::Mat& Intensity,
        int d_min,
        int n_samples_per_octave,
        int n_octaves,
        double positive_pulse_width,
        double negative_pulse_width,
        int window_width, // w
        int center_of_image_row // uc
) {
    std::pair<int, int> best_pair;
    int image_height = Intensity.size[0];
    int period = 0;
    // int center = window_width / 2; TODO useme
    std::pair<int, int> x;
    int energy = 0;
    std::vector<std::pair<int,int>> best_pairs;

    int n_frequencies = (n_samples_per_octave * n_octaves) + 1; // actually a period
    for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
        //std::cout << "row: " << image_row_num << std::endl;
        cv::Mat intensity_row = Intensity.row(image_row_num);
        int best_energy = 0;
        for (int k = 0; k < n_frequencies - 1; k++) {
            period = d_min * std::pow(2, k / n_samples_per_octave);
            //std::cout << "frequency: " << 1.0/period << std::endl;

            int half_band = (int) std::round(0.5 * period);
            for (int phase = -half_band; phase < half_band; phase++) {
                x = std::make_pair(phase, period);
                energy = CrossCorrelation(
                        intensity_row,
                        x,
                        positive_pulse_width, negative_pulse_width,
                        window_width, center_of_image_row);

                // std::cout << "phase: " << x.first << " period: " << x.second << " energy: " << energy << std::endl;
                if(energy > best_energy){
                    best_energy = energy;
                    best_pair = x;
                }
            }
        }
        best_pairs.push_back(best_pair);
    }
    return best_pairs;
}
int CropRowDetector::CrossCorrelation(cv::Mat I, std::pair<int, int> template_var_param,
                     int positive_pulse_width, int negative_pulse_width,
                     int image_width, int center_of_row){
    int phase = template_var_param.first;
    int period = template_var_param.second;
    int positive_pulse_start, positive_pulse_end, negative_pulse_start, negative_pulse_end;
    int pulse_center = center_of_row + phase - (int)std::round ((center_of_row + phase) / period ) * period;
    int positive_correlation_value = 0;
    int negative_correlation_value = 0;
    int positive_pixels = 0;
    int negative_pixels = 0;
    do{
        positive_pulse_start = std::round(pulse_center - positive_pulse_width/2);
        positive_pulse_end = positive_pulse_start + positive_pulse_width;
        negative_pulse_start = std::round(pulse_center + period/2 - negative_pulse_width/2);
        negative_pulse_end = negative_pulse_start + negative_pulse_width;
        positive_pulse_start = saturate(positive_pulse_start, 0, image_width);
        positive_pulse_end = saturate(positive_pulse_end, 0, image_width);
        negative_pulse_start = saturate(negative_pulse_start, 0, image_width);
        negative_pulse_end = saturate(negative_pulse_end, 0, image_width);
        positive_correlation_value = positive_correlation_value + cumulative_sum(I, positive_pulse_end) - cumulative_sum(I, positive_pulse_start);
        negative_correlation_value = negative_correlation_value + cumulative_sum(I, negative_pulse_end) - cumulative_sum(I, negative_pulse_start);
        positive_pixels += positive_pulse_end - positive_pulse_start;
        negative_pixels += negative_pulse_end - negative_pulse_start;
        pulse_center += period;
    } while(pulse_center - positive_pulse_width/2 <= image_width);

    int positive_pulse_height;
    if(positive_pixels != 0) {
        positive_pulse_height = 1 / positive_pixels;
    } else {
        positive_pulse_height = 0;
        // std::cout << "no positive pulse matched!!!!" << std::endl;
    }

    int negative_pulse_height;
    if(negative_pixels != 0){
        negative_pulse_height = 1/negative_pixels;
    } else {
        negative_pulse_height = 0;
        // std::cout << "no negative pulse matched!!!!" << std::endl;
    }
    // f[v][x] = z_a * f_a - z_b * f_b;
    // return f[v][x];
    return positive_pulse_height * positive_correlation_value - negative_pulse_height * negative_correlation_value;
}
/*
std::pair<int, int> CropRowDetector::find_optimal_x(std::vector<int> f, X, h, x){
    for(int v=0; v < h; v++){
        f_max[v] = std::max(f[v]);
        for(std::pair<int, int> x: X){
            if(f_max[v] >= f_low){
                D[v][x] = min( (1-f[v][x]) / f_max[v], D_max);
            } else {
                D[v][x] = D_max;
            }
            if(v != 0){
                B[v][x] = D[v][x] + U[v-1][x];
            } else{
                B[v][x] = D[v][x];
            }
        }
        if(v < h-1){
            for(std::pair<int, int> x: X) {
                for(std::pair<int, int> x_prime: X) {
                    U_prime[v][x][x_prime] = B[v][x_prime] + V(x, x_prime);
                    T_prime[v][x] = B[v][x_prime] + V(x_prime, x);
                }
                U[v][x] = min(U_prime[v][x]);
                T[v][x] = argmin(T_prime[v][x]);
            }
        }
    }
}
*/

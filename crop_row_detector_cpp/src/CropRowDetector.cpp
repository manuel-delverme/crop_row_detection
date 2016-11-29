//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
#include <jpegint.h>
#include "CropRowDetector.h"

using namespace std;

CropRowDetector::CropRowDetector(cv::Mat const intensity_map) {



    cv::Mat intensity_map_64f;
    intensity_map.convertTo(intensity_map_64f, CV_64F);
    m_integral_image = cv::Mat::zeros( intensity_map.size(), CV_64F );

    for(int row = 0; row < intensity_map.rows; row++) {
        for (int column = 1; column < intensity_map.cols; column++){
            m_integral_image.at<double>(row,column) = (double)intensity_map.at<uchar>(row,column) + m_integral_image.at<double>(row,column-1);

            //cout << m_integral_image.row(row).col(column) << endl;
        }
    }

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
        intensity.row(i).convertTo(plot_image, CV_64F);
        average_intensity += (plot_image / intensity.rows);
    }
    std::cout << "displaying avg" << std::endl;
    plot = cv::plot::createPlot2d(average_intensity);
    plot->render(plot_result);
    cv::imshow("row intensity", plot_result);
    cv::waitKey(0);
    return result;
}

/*
 * returns the best pair x for each row
 * */
std::vector<std::pair<int, int>> CropRowDetector::template_matching(
        const cv::Mat& Intensity,
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
    double energy = 0;
    std::vector<std::pair<int,int>> best_pairs;

    int n_samples = (n_samples_per_octave * n_octaves);
    double best_energy = -1;

    for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
        //std::cerr << "row: " << image_row_num << std::endl;


        for (int sample_number = 0; sample_number <= n_samples; sample_number++) { // periods
            period = (int) (d_min * std::pow(2, (double) sample_number / (double) n_samples_per_octave));
            int half_band = (int) std::round(0.5 * period);


            for (int phase = -half_band; phase < half_band; phase++) { // phase

                x = std::make_pair(phase, period);
                energy = CrossCorrelation(
                        image_row_num, x,
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
        best_energy = -1;
    }
    return best_pairs;
}
double CropRowDetector::CrossCorrelation(int row_number, std::pair<int, int> template_var_param,
                                         double positive_pulse_width, double negative_pulse_width,
                                         int image_width, int center_of_row){

    // here be dragons
    // for(v = 0; v < m_h; v++, I_ += w, DP_ += n)
    // {
    // integral image of the row
    // II[0] = (int)(I_[0]);

    // for(u = 1; u < w; u++)
    //     II[u] = II[u - 1] + I_[u];

    // convolutions

    // bestScore = 0.0;

    // d = (double)8;
    // DP__ = DP_;
    // for(id = 0; id < m_nd; id++, d *= m_dstep, DP__ += m_nc)
    // {
    // crange = (int)floor(0.5 * d);
    // crange_[id] = crange;

    // scale = d / fd0;
    // fTmp = floor(scale * fa0 + 0.5);
    // a = (int)fTmp;
    // halfa = 0.5 * fTmp;
    // fTmp = floor(scale * fb0 + 0.5);
    // b = (int)fTmp;
    // halfb = 0.5 * fTmp;
    // halfd = 0.5 * d;
    // fub =  halfd - halfb;
    // pDP = DP__ + m_nc / 2 - crange;
    double phase = template_var_param.first;
    double period = template_var_param.second;


    int negative_pulse_end;
    int negative_pulse_start;
    int positive_pulse_end;
    int positive_pulse_start;
    double pulse_center = center_of_row + phase - std::ceil ((center_of_row + phase) / period ) * period;
    double positive_correlation_value = 0;
    double negative_correlation_value = 0;
    int positive_pixels = 0;
    int negative_pixels = 0;

    int offset_first_wave = (int)floor((-(image_width / 2 + phase) + halfa) / period);
    int offset_last_wave = (int)floor(((image_width / 2 - phase) + halfa) / period);

    pulse_center = phase + (double)offset_first_wave * period + halfw;
    positive_pulse_start = DOUBLE2INT(pulse_center - halfa);
    positive_pulse_end = (int) (positive_pulse_start + positive_pulse_width - 1); // TODO check why he was not casting

    if(positive_pulse_end >= 0) {
        positive_correlation_value = cumulative_sum(row_number, 0, positive_pulse_end);
        positive_pixels = positive_pulse_end;
    } else {
        positive_correlation_value = 0;
        positive_pixels = 0;
    }

    negative_pulse_start = DOUBLE2INT(pulse_center + fub);
    negative_pulse_end = (int) (negative_pulse_start + negative_pulse_width - 1);

    if(negative_pulse_start < 0)
        negative_pulse_start = 0;

    if(negative_pulse_end >= 0)
    {
        negative_correlation_value = cumulative_sum(row_number, negative_pulse_start - 1, negative_pulse_end);
        negative_pixels = negative_pulse_end - negative_pulse_start + 1;
    }
    else
    {
        negative_correlation_value = 0;
        negative_pixels = 0;
    }

    pulse_center += period;

    for(k = offset_first_wave + 1; k < offset_last_wave; k++, pulse_center += period)
    {
        positive_pulse_start = DOUBLE2INT(pulse_center - halfa);

        positive_pulse_end = positive_pulse_start + positive_pulse_width - 1;

        positive_correlation_value += cumulative_sum(row_number, positive_pulse_start - 1, positive_pulse_end);
        positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

        negative_pulse_start = DOUBLE2INT(pulse_center + fub);

        negative_pulse_end = negative_pulse_start + negative_pulse_width - 1;

        negative_correlation_value += cumulative_sum(row_number, negative_pulse_start - 1, negative_pulse_end);
        negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
    }

    positive_pulse_start = DOUBLE2INT(pulse_center - halfa);

    positive_pulse_end = positive_pulse_start + positive_pulse_width - 1;

    if(positive_pulse_end >= image_width)
        positive_pulse_end = image_width - 1;

    positive_correlation_value += cumulative_sum(row_number, positive_pulse_start - 1, positive_pulse_end);
    positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

    negative_pulse_start = std::round(pulse_center + fub);

    if(negative_pulse_start < image_width)
    {
        negative_pulse_end = negative_pulse_start + negative_pulse_width - 1;
        if(negative_pulse_end >= image_width)
            negative_pulse_end = image_width - 1;
        negative_correlation_value += cumulative_sum(row_number, negative_pulse_start - 1, negative_pulse_end);
        negative_pixels += (negative_pulse_end - negative_pulse_start + 1);

        //debugCounter++;
    }
    //score = (double)(nb * Sa - na * Sb) / (double)(na + nb);
    score = (double)(negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) / (double)(positive_pixels * negative_pixels);		// new score computation

    //END IMAGE

    //copy D value for c values in range <-m_nc/2, m_nc/2>
    crange2 = 2*crange;
    pDP_ = pDP + crange2; //pDP_ pointer for copying D value in c range <-m_nc/2, m_nc/2>
    c_position = m_nc*0.5 + phase + crange2;

    while(c_position <= m_nc)
    {
        pDP_->cost = score;

        pDP_ += crange2;
        c_position += crange2;
    }

    pDP_ = pDP - crange2;
    c_position = m_nc*0.5 + phase - crange2;

    while(c_position >= 0)
    {
        pDP_->cost = score;

        pDP_ -= crange2;
        c_position -= crange2;
    }

    if(score > bestScore)
    {
        bestScore = score;
        bestc = phase;
        bestid = id;
        bestd = period;
    }

    do{
        positive_pulse_start = (int) (pulse_center - positive_pulse_width / 2);
        positive_pulse_end = (int)(positive_pulse_start + positive_pulse_width - 1); // TODO: EXPLAIN -1
        negative_pulse_start = (int) (pulse_center + period / 2 - negative_pulse_width / 2);
        negative_pulse_end = (int)(negative_pulse_start + negative_pulse_width - 1);
        positive_pulse_start = saturate(positive_pulse_start, 0, image_width - 1);
        positive_pulse_end = saturate(positive_pulse_end, 0, image_width - 1);
        negative_pulse_start = saturate(negative_pulse_start, 0, image_width - 1);
        negative_pulse_end = saturate(negative_pulse_end, 0, image_width - 1);

        positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                      - cumulative_sum(row_number, positive_pulse_start);

        negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                      -cumulative_sum(row_number, negative_pulse_start);

        positive_pixels += positive_pulse_end - positive_pulse_start;
        negative_pixels += negative_pulse_end - negative_pulse_start;
        pulse_center += period;
    } while(pulse_center - positive_pulse_width/2 <= image_width);

    float positive_pulse_height;
    if(positive_pixels != 0) {
        positive_pulse_height = 1.f / positive_pixels;
    } else {
        positive_pulse_height = 0.f;
    }

    float negative_pulse_height;
    if(negative_pixels != 0){
        negative_pulse_height = 1.f/negative_pixels;
    } else {
        negative_pulse_height = 0.f;
    }

    return positive_pulse_height * positive_correlation_value - negative_pulse_height * negative_correlation_value;
}



double CropRowDetector::cumulative_sum(int v, int start, int end) {
    return m_integral_image.at<double>(v, end) - m_integral_image.at<double>(v, start);
}

std::pair<int, int> CropRowDetector::find_optimal_x(std::vector<double> energy, X, num_rows, x, double f_low=1, double D_max){
    for(int row_number=0; row_number < num_rows; row_number++){
        max_energy[row_number] = std::max(energy.at<double>(row_number));
        for(std::pair<int, int> x: X){
            if(max_energy[row_number] >= f_low){
                cost[row_number][x] =
                        std::min((1.0 -
                                         energy.at<double>(row_number, x)) /
                                         max_energy[row_number],
                                  D_max);
            } else {
                cost[row_number][x] = D_max;
            }
            if(row_number > 0){
                best[row_number][x] = cost[row_number][x] + U[row_number-1][x];
            } else{
                best[row_number][x] = cost[row_number][x];
            }
        }
        if(row_number < h-1){
            for(std::pair<int, int> x: X) {
                for(std::pair<int, int> x_prime: X) {
                    U_prime[row_number][x][x_prime] = best[row_number][x_prime] + V(x, x_prime);
                    T_prime[row_number][x] = best[row_number][x_prime] + V(x_prime, x);
                }
                U[row_number][x] = min(U_prime[row_number][x]);
                T[row_number][x] = argmin(T_prime[row_number][x]);
            }
        }
    }
}

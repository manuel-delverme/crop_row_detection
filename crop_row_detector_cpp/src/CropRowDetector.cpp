//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
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
double CropRowDetector::CrossCorrelation(int row, std::pair<int, int> template_var_param,
                                         double positive_pulse_width, double negative_pulse_width,
                                         int image_width, int center_of_row){
    int phase = template_var_param.first;
    int period = template_var_param.second;
    int negative_pulse_end;
    int negative_pulse_start;
    int positive_pulse_end;
    int positive_pulse_start;
    double pulse_center = center_of_row + phase - std::ceil ((center_of_row + phase) / period ) * period;
    double positive_correlation_value = 0;
    double negative_correlation_value = 0;
    int positive_pixels = 0;
    int negative_pixels = 0;
    
    
    do{

        positive_pulse_start = std::round(pulse_center - positive_pulse_width / 2);
	positive_pulse_end = (int)(positive_pulse_start + positive_pulse_width);
	negative_pulse_start = std::round(pulse_center + period / 2 - negative_pulse_width / 2);
	negative_pulse_end = (int)(negative_pulse_start + negative_pulse_width);
	positive_pulse_start = saturate(positive_pulse_start, 0, image_width - 1);
	positive_pulse_end = saturate(positive_pulse_end, 0, image_width - 1);
	negative_pulse_start = saturate(negative_pulse_start, 0, image_width - 1);
	negative_pulse_end = saturate(negative_pulse_end, 0, image_width - 1);   
    
        positive_correlation_value += cumulative_sum(row, positive_pulse_end)
                                      - cumulative_sum(row, positive_pulse_start);
				      
        negative_correlation_value += cumulative_sum(row, negative_pulse_end)
                                      -cumulative_sum(row, negative_pulse_start);

        positive_pixels += positive_pulse_end - positive_pulse_start;
        negative_pixels += negative_pulse_end - negative_pulse_start;
        pulse_center += period;
    } while(pulse_center - positive_pulse_width/2 <= image_width);
    
    int positive_pulse_height;
    if(positive_pixels != 0) {
        positive_pulse_height = 1 / positive_pixels;
    } else {
        positive_pulse_height = 0;
    }

    int negative_pulse_height;
    if(negative_pixels != 0){
        negative_pulse_height = 1/negative_pixels;
    } else {
        negative_pulse_height = 0;
    }
        
    return positive_pulse_height * positive_correlation_value - negative_pulse_height * negative_correlation_value;
}



double CropRowDetector::cumulative_sum(int v, int u) {
    return m_integral_image.at<double>(v, u);
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

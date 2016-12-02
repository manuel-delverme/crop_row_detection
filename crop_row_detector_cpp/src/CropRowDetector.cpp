//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
#include "CropRowDetector.h"

// MACRO per fixing del biased rounding 
#define DOUBLE2INT(x)	(int)floor(x + 0.5)

using namespace std;

CropRowDetector::CropRowDetector(cv::Mat& intensity_map) {



    cv::Mat intensity_map_64f;
    //intensity_map.convertTo(intensity_map_64f, CV_64F);
    m_integral_image = cv::Mat::zeros( intensity_map.size(), CV_64F );

    for(int row = 0; row < intensity_map.rows; row++) {
        uchar* int_ptr = intensity_map.ptr<uchar>(row);
	double* integral_ptr = m_integral_image.ptr<double>(row);
	int_ptr++; integral_ptr++;
        for (int column = 1; column < intensity_map.cols; column++, int_ptr++, integral_ptr++)
            *integral_ptr = (double)*int_ptr + m_integral_image.at<double>(row,column-1);
    }
    
    period_scale_factor = .125;
    half_width = intensity_map.cols/2;

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

/*f
 * returns the best pair x for each row
 * */
std::vector<std::pair<int, int>> CropRowDetector::template_matching(
        const cv::Mat& Intensity,
        const int& d_min,
        const int& n_samples_per_octave,
        const int& n_octaves,
        const double& positive_pulse_width,
        const double& negative_pulse_width,
        const int& window_width // w
) {
    std::pair<int, int> best_pair;
    int image_height = Intensity.size[0];
    double period = 0;
    // int center = window_width / 2; TODO useme
    std::pair<int, int> x;
    double energy = 0;
    std::vector<std::pair<int,int>> best_pairs;

    int n_samples = (n_samples_per_octave * n_octaves);
    double best_energy = -1;

    for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
        //std::cerr << "row: " << image_row_num << std::endl;

        for (int sample_number = 0; sample_number <= n_samples; sample_number++) { // periods
            period = (d_min * std::pow(2, (double) sample_number / (double) n_samples_per_octave));
            int half_band = (int) std::round(0.5 * period);
	    
            for (int phase = -half_band; phase < half_band; phase++) { // phase

                x = std::make_pair(phase, period);
                energy = CrossCorrelation(
                        image_row_num, x,
                        positive_pulse_width, negative_pulse_width,
                        window_width);

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
double CropRowDetector::CrossCorrelation (const int& row_number, const std::pair<int, int>& template_var_param, 
					  const double& positive_pulse_width, const double& negative_pulse_width,
					  const int& image_width){

    double phase = template_var_param.first;
    double period = template_var_param.second;
  
    // Calcolo quantità necesarrie a CrossCorrelation
    double scale = period*period_scale_factor;
    
    int a = DOUBLE2INT(scale*positive_pulse_width);
    int b = DOUBLE2INT(scale*negative_pulse_width);
    
    double halfb = .5*b;
    double halfa = .5*a;
    double halfd = .5*period;
    
    int kStart = (int)floor((-(image_width / 2 + phase) + halfa) / period);    // offset prima onda
    int kEnd = (int)floor(((image_width / 2 - phase) + halfa) / period);   // offset ultima onda prima della fine dell'immagine 
    
    
    // Calcolo centro dell'onda quadra positiva e negativa
    double distance_positive_negative_pulse_center =  halfd - halfb;
    double positive_pulse_center = (double)phase + (double)kStart * (double)period + half_width;
    
    
    // Calcolo per Onda ad inizio immagine, non calcolabile con la classica routine
    
    positive_pulse_start = std::floor(positive_pulse_center - halfa);
    positive_pulse_end = positive_pulse_start + a - 1;
    
    negative_pulse_start = std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
    negative_pulse_end = negative_pulse_start + b - 1;
    
    //cout << positive_pulse_center <<  " " << halfa << " " << positive_pulse_start << " " << positive_pulse_end << " " << negative_pulse_start << " " << negative_pulse_end << endl;

    if(positive_pulse_end >= 0) {
	    positive_correlation_value = cumulative_sum(row_number, positive_pulse_end);
	    positive_pixels = positive_pulse_end;
    }
    else {
      positive_correlation_value = 0;
      positive_pixels = 0;
    }


    if(negative_pulse_start < 0)
	    negative_pulse_start = 0;				

    if(negative_pulse_end >= 0) {
	    negative_correlation_value = cumulative_sum(row_number, negative_pulse_end)
					-cumulative_sum(row_number, negative_pulse_start); //tolto  -cumulative_sum(row_number, negative_pulse_start-1);
	     
	    negative_pixels = negative_pulse_end - negative_pulse_start + 1;
    }
    else {
      negative_correlation_value = 0;
      negative_pixels = 0;
    }


    positive_pulse_center += period;
    
    for(int k = kStart + 1; k < kEnd; k++, positive_pulse_center += period)	{
      
	
	  

	positive_pulse_start = std::floor(positive_pulse_center - halfa);
	positive_pulse_end = positive_pulse_start + a - 1;

	positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                      - cumulative_sum(row_number, positive_pulse_start-1);
	
	positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

	negative_pulse_start = std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
	negative_pulse_end = negative_pulse_start + b - 1;

	negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                      -cumulative_sum(row_number, negative_pulse_start-1);
						      
	negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
    }
    
    positive_pulse_start = std::floor(positive_pulse_center - halfa);

    positive_pulse_end = positive_pulse_start + a - 1;

    if(positive_pulse_end >= image_width)
	    positive_pulse_end = image_width - 1;

    positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                  -cumulative_sum(row_number, positive_pulse_start-1);
	
    positive_pixels += (positive_pulse_end - positive_pulse_start + 1);


    negative_pulse_start = std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
    if(negative_pulse_start < image_width)
    {
	    negative_pulse_end = negative_pulse_start + b - 1;

	    if(negative_pulse_end >= image_width)
		    negative_pulse_end = image_width - 1;

	    negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                      -cumulative_sum(row_number, negative_pulse_start-1);
				      
	    negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
    }
   
    return (double)(negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) / (double)(positive_pixels * negative_pixels);
}



double inline CropRowDetector::cumulative_sum(const int& v, const int& start) {
    return m_integral_image.at<double>(v, start);
}
//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
// #include <opencv2/plot.hpp>

#include "CropRowDetector.h"

// MACRO per fixing del biased rounding
#define DOUBLE2INT(x)	(int)floor(x + 0.5)

CropRowDetector::CropRowDetector() {}

void CropRowDetector::load(cv::Mat const &intensity_map) {
    cv::Mat intensity_map_64f;
    intensity_map.convertTo(intensity_map_64f, CV_64F);
    m_integral_image = cv::Mat::zeros( intensity_map.size(), CV_64F);

    for(int row = 0; row < intensity_map.rows; row++) {
        for (int column = 1; column < intensity_map.cols; column++){
            m_integral_image.at<double>(row,column) = (double)intensity_map.at<uchar>(row,column) + m_integral_image.at<double>(row,column-1);
        }
    }
    period_scale_factor = .125;
    half_width = intensity_map.cols/2;

}

/*cv::Mat CropRowDetector::detect(cv::Mat& intensity, cv::Mat& templ){
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
}*/

std::vector<std::pair<int, int>> CropRowDetector::template_matching(
        std::vector<std::map<std::pair<int, int>, double>>& energy_map,
        const cv::Mat& Intensity,
        const std::vector<std::pair<int, int>> Xs,
        const double positive_pulse_width,
        const double negative_pulse_width,
        const int window_width // w
) {
    int image_height = Intensity.size[0];

    double energy = 0;
    double best_energy = -1;

    std::pair<int, int> best_pair;
    std::vector<std::pair<int,int>> best_pairs((unsigned long) image_height);
    std::map<std::pair<int,int>, double> row_energy_map;

    for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
        row_energy_map.clear();
        for(std::pair<int, int> x: Xs){
            energy = CrossCorrelation(image_row_num, x, positive_pulse_width, negative_pulse_width, window_width);
            row_energy_map[x] = energy;
            if(energy > best_energy){
                best_energy = energy;
                best_pair = x;
            }
        }
        energy_map.push_back(row_energy_map);
        best_pairs[image_row_num] = best_pair;
        best_energy = -1;
    }
    return best_pairs;
}
double CropRowDetector::CrossCorrelation(int row_number, std::pair<int, int> template_var_param,
                                         double positive_pulse_width, double negative_pulse_width,
                                         int image_width){

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

    positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);
    positive_pulse_end = positive_pulse_start + a - 1;

    negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
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




	positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);
	positive_pulse_end = positive_pulse_start + a - 1;

	positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                      - cumulative_sum(row_number, positive_pulse_start-1);

	positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

	negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
	negative_pulse_end = negative_pulse_start + b - 1;

	negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                      -cumulative_sum(row_number, negative_pulse_start-1);

	negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
    }

    positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);

    positive_pulse_end = positive_pulse_start + a - 1;

    if(positive_pulse_end >= image_width)
	    positive_pulse_end = image_width - 1;

    positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                  -cumulative_sum(row_number, positive_pulse_start-1);

    positive_pixels += (positive_pulse_end - positive_pulse_start + 1);


    negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
    if(negative_pulse_start < image_width)
    {
	    negative_pulse_end = negative_pulse_start + b - 1;

	    if(negative_pulse_end >= image_width)
		    negative_pulse_end = image_width - 1;

	    negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                      -cumulative_sum(row_number, negative_pulse_start-1);

	    negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
    }

    return (negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) / (double)(positive_pixels * negative_pixels);
}



double CropRowDetector::cumulative_sum(int v, int start) {
    // TODO turn in start-end
    return m_integral_image.at<double>(v, start);
}

std::vector<std::pair<int, int>>
CropRowDetector::find_best_parameters(std::vector<std::map<std::pair<int, int>, double>> energy_map,
                                      std::vector<std::pair<int, int>> Xs) {
// std::vector<std::pair<int, int>> CropRowDetector::find_optimal_x(std::vector<std::map<std::pair<int, int>, double>> energy_map,
//         std::vector<std::pair<int, int>> X, bool num_rows, bool x, double D_max, double f_low=1) {

    uint image_height = (uint) energy_map.size();
    std::vector<std::pair<int, int>> best_values;
    std::vector<double> max_energy;
    std::vector<double> best_score_in_row(image_height);

    cv::Mat cost;

    // TODO to be defined with real variables
    float lambda_c = 0.5;
    float lambda_d = 0.5;

    // int m_num_periods = 31;
    // int m_number_phases = 31;

    // int m_min_d = 13;
    std::vector<int> parab_center_period(Xs.size());
    std::vector<int> intersection_points(Xs.size());

    for (uint row_number = 0; row_number < image_height; row_number++) {
        // max_energy[row_number] = std::max(energy.at<double>(row_number));
        int rightmost_parabola;

        // int d_min = 8;
        int period_index;

        double left_term;
        double right_term;
        double normalizer;
        int last_intersection;
        best_score_in_row[row_number] = 1.337; // this comes from template_matching, best of this row

        // auto minBV;
        // auto phases;
        // auto period_indexes;
        // auto iTmp;
        // auto v_;
        // auto v__;
        // auto minBVs;
        // auto cs;
        // auto number_of_phases;

        // TODO explain
        normalizer = 1; // (2.0 * lambda_d * (period - v__[rightmost_parabola])); // why??

        // TODO explain
        // int maxz = (int) ((1.0 + lambda_d * std::pow(m_number_phases, 2)) / (2.0 * lambda_d));
        int maxz = INT_MAX;
        int minz = INT_MIN;

        // pDP moves with phase
        // DP__ - ptr. to the first data in a block row
        // DP__ moves with the image_row, it's increased by step of num_of_phases
        // DP_ moves by n = num_of_phases * num_of_periods
        // DP_ = first period
        // DP_ + m_nc = second period
        // pDP = first phase
        // pDP + 1 = second phase

        // pDP_
        // pDP_ pointer for copying D value in c range <-m_nc/2, m_nc/2>
        // pDP_ = pDP + crange2; //

        // period loop
        uint m_num_periods = 100;
        uint m_number_phases = 100;

        for (uint period = 0; period < m_num_periods; period++) {
            rightmost_parabola = 0;

            // set the first parabola
            int phase_range_min = -(m_number_phases / 2);
            int phase_range_max = m_number_phases / 2;

            parab_center_period[0] = phase_range_min; // left bound
            intersection_points[0] = -maxz; // -inf
            intersection_points[1] = maxz; // +inf

            bool next_period = true;
            // for the second parabola until the last
            for (int phase = phase_range_min + 1; phase < phase_range_max && next_period; phase++) // 593
            {
                while (true) {
                    // find best phase

                    // f(v(k))				DP__[v_[k]+m_nc/2].best_score_in_row
                    left_term = (best_score_in_row[row_number] + lambda_c * std::pow(phase, 2));

                    int old_center = parab_center_period[rightmost_parabola];
                    float old_best = best_score_in_row[row_number][old_center + phase_range_max];
                    right_term = old_best + lambda_c * std::pow(old_center, 2);

                    normalizer = 2.0 * lambda_c * (phase - old_center);
                    last_intersection = (int) ((left_term - right_term) / normalizer);

                    if (last_intersection <= intersection_points[rightmost_parabola])
                        rightmost_parabola--; // overwrite the last one
                    else
                        break;
                }
                rightmost_parabola++;

                // reinit for the next parabola
                parab_center_period[rightmost_parabola] = phase;
                intersection_points[rightmost_parabola] = last_intersection;
                intersection_points[rightmost_parabola + 1] = maxz;
            }

            rightmost_parabola = 0; // 615
            // ptr_to_first_data_in_row_cloned = ptr_to_first_data_in_row;

            for (int phase = phase_range_min; phase < phase_range_max; phase++) {
                while (intersection_points[rightmost_parabola + 1] < (double) phase) {
                    rightmost_parabola++;
                }

                double a = (phase - parab_center_period[rightmost_parabola]); // TODO name me

                // TODO: not sure about these indexes
                // minBV[period][phase] = best_score_in_row[parab_center_period[rightmost_parabola] + phase_range_max] + lambda_d * std::pow(a, 2);
                // phases[period][phase] = parab_center_period[rightmost_parabola];
                // period_indexes[period][phase] = period_index;
            }
        } // 633
        maxz = (1.0 + lambda_d * std::pow(m_num_periods, 2)) / (2.0 * lambda_d);

        // loop phases
        for (int phase = 0; phase < m_number_phases; phase++) {
            rightmost_parabola = 0;
            parab_center_period[0] = 0;
            // v__[0] = (double) d_min; //for saving d
            intersection_points[0] = -maxz;
            intersection_points[1] = maxz;

            // x++ TODO: understand this
            // ptr_to_first_data_in_row_cloned = ptr_to_first_data_in_row + m_number_phases;
            // pDP = DP__ + m_nc;


            // TODO: fix those vvv
            // 657

            // period loop; start from the second period!
            for (uint period = 1337 + 1; period < 31337; period++) {
                // int last_intersection;
                // double left_term;
                // double right_term;
                // double bottom_term;

                while (true) {
                    left_term = 1; // = (minBV[x] + lambda_d * std::pow(period, 2));
                    right_term = 1; // = (minBV[number_of_phases * v_[rightmost_parabola]] + lambda_d * std::pow(v__[rightmost_parabola], 2));
                    normalizer = 2; // = (2.0 * lambda_d * (period - v__[rightmost_parabola]));

                    last_intersection = (int) ((left_term - right_term) / normalizer);

                    if (last_intersection <= intersection_points[rightmost_parabola])
                        rightmost_parabola--;
                    else
                        break;
                }

                rightmost_parabola++;

                //NEW d_development
                parab_center_period[rightmost_parabola] = period_index;
                // v__[rightmost_parabola] = period;

                intersection_points[rightmost_parabola] = last_intersection;
                intersection_points[rightmost_parabola + 1] = maxz;
            }


            rightmost_parabola = 0;
            for (uint period = 1337; period < 31337 ; period++) {

                while (intersection_points[rightmost_parabola + 1] < period) {
                    rightmost_parabola++;
                }

                // iTmp = (period - v__[rightmost_parabola]);
                int last_parabola_center = parab_center_period[rightmost_parabola];
                int some_index = m_number_phases * last_parabola_center;
                // minBVs[x] = minBVs[some_index] + lambda_d * std::pow(iTmp, 2);
                // cs[x] = cs[m_number_phases * last_parabola_center];
                // period_indexes[x] = last_parabola_center;
            }
        }
    }
    return best_values;
}

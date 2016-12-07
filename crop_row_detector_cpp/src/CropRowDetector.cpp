//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
// #include <ppmdraw.h>

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
    std::vector<std::pair<int,int>> best_pairs;

    for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
        for(std::pair<int, int> x: Xs){
            energy = CrossCorrelation(image_row_num, x, positive_pulse_width, negative_pulse_width, window_width);
            energy_map[image_row_num][x] = energy;
            if(energy > best_energy){
                best_energy = energy;
                best_pair = x;
            }
        }
        best_pairs.push_back(best_pair);
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

std::pair<int, int> CropRowDetector::find_optimal_x( std::vector<std::map<std::pair<int, int>, double>> energy_map,
        bool X, bool num_rows, bool x, double D_max, double f_low=1) {

    std::vector<double> max_energy;
    cv::Mat cost;
    for (int row_number = 0; row_number < num_rows; row_number++) {
        // max_energy[row_number] = std::max(energy.at<double>(row_number));

        /*
        for (std::pair<int, int> x: X) // 424 period_loop
        {
            for (int phase = -10; phase < 100; phase++) { // 428 phase loop
                if (max_energy[row_number] >= f_low) {
                    float cost = (1.0 - energy.at<double>(row_number, x)) / max_energy[row_number];
                    cost.at<float>(row_number, x) = std::min(cost, D_max);
                } else {
                    cost[row_number][x] = D_max;
                }
                if (row_number > 0) {
                    best[row_number][x] = cost[row_number][x] + U[row_number - 1][x];
                } else {
                    best[row_number][x] = cost[row_number][x]; // can be skipped by directly assigning to best
                }
            }
            if (row_number < number_of_rows - 1) {  // 445
                std::pair<int, int> old_x = X[0];
                for (int period : periods){ // 449 period_loop
                    // calc phase 0
                    min_bv[x] = best[x];
                    c[x] = -crange;
                    period_index[x] = period_index;
                    old_x = x;
                    phase++;
                    for(int phase; phase < crange; phase++) // 464
                        minBV[this_x] = B[this_x];
                        BV = min_bv[old_x] + m_lambda_d;
                        if (min_bv[min_bv] <= BV) {
                            c[x] = phase;
                            period_index[x] = period_index;
                        } else {
                            minBV[x] = BV;
                            c[x] = c[old_x];
                            period_index[x] = period_index[old_x];
                        }
                        old_x = x;
                    }
                    for (; phase < m_number_phases; phase++, x++) { // 482
                        minBV[x] = minBV[old_x] + m_lambda_d;
                        c[x] = c[old_x];
                        period_index[x] = period_index[old_x];
                        old_x = x;
                    }

                    old_x = ptr_to_first_data_in_row + m_number_phases / 2 + crange - 1;
                    x = p_old_x - 1;

                    for (phase = crange - 2; phase >= -crange; phase--, x--) { // 495
                        BV = minBV[old_x] + m_lambda_d;

                        if (minBV[x] > BV) {
                            // p_this_x->minBV = BV;
                            minBV[x] = BV;
                            c[x] = c[old_x];
                        }
                        old_x = x;
                    }
                    for (; phase >= -m_number_phases / 2; phase--, x--) { // 509
                        minBV[x] = minBV[old_x] + m_lambda_d;
                        c[x] = c[old_x];
                        period_index[x] = period_index[old_x];

                        old_x = x;
                    }
                }
                // reset the pointer
                // ptr_to_first_data_in_row = DP_;

                for (phase = 0; phase < m_number_phases; phase++, ptr_to_first_data_in_row++) { // 522
                    p_old_x = ptr_to_first_data_in_row;

                    p_this_x = p_old_x + m_number_phases;

                    for (period_index = 1; period_index < m_num_periods; id++, p_this_x += m_number_phases) { // 528
                        BV = p_old_x->minBV + m_lambdad;

                        if (p_this_x->minBV > BV) { // 532
                            p_this_x->minBV = BV;
                            p_this_x->c = p_old_x->c;
                            p_this_x->period_index = p_old_x->period_index;
                        }

                        p_old_x = p_this_x;
                    }

                    p_old_x = ptr_to_first_data_in_row + (m_num_periods - 1) * m_number_phases;

                    p_this_x = p_old_x - m_number_phases;

                    for (period_index = m_num_periods - 2; period_index >= 0; period_index--, p_this_x -= m_number_phases) { // 546
                        BV = p_old_x->minBV + m_lambdad;

                        if (p_this_x->minBV > BV) { // 550
                            p_this_x->minBV = BV;
                            p_this_x->c = p_old_x->c;
                            p_this_x->period_index = p_old_x->period_index;
                        }

                        p_old_x = p_this_x;
                        crange = crange_[period_index];
                }    // for(c = 0; c < m_nc; c++, DP_++)
                */

        int rightmost_parabola;

        // TODO to be defined with real variables
        auto parab_center;
        auto intersection_point;
        auto m_num_periods;
        auto m_number_phases = 31337;

        auto m_min_d = 31;
        auto lambda_c = 31.37;
        auto lambda_d = 31.37;
        auto period_index = 31337;
        auto minBV;
        auto phases;
        auto period_indexes;
        auto iTmp;
        auto v_;
        auto v__;
        auto minBVs;
        auto cs;
        auto number_of_phases;
        auto d_min;
        auto period;

        double c = (2.0 * lambda_d * (period - v__[rightmost_parabola]));

        double maxz = (1.0 + lambda_d * (double) (m_number_phases * m_number_phases)) / (2.0 * lambda_d);
        std::vector<std::vector<float>> B;

        // period loop
        for (uint period = 0; period < m_num_periods; period++) {
            rightmost_parabola = 0;

            // set the first parabola
            int phase_range_min = -m_number_phases / 2;
            int phase_range_max = m_number_phases / 2;

            parab_center[0] = phase_range_min; // left bound
            intersection_point[0] = -maxz; // -inf
            intersection_point[1] = maxz; // +inf

            // for the second parabola until the last
            for (int phase = phase_range_min + 1; phase < phase_range_max; phase++) // 593
            {
                double last_intersection;
                while (true) {
                    // find best phase

                    double a = (B[x] + lambda_c * std::pow(phase, 2)); // TODO name me
                    double b = (B[parab_center[rightmost_parabola] + phase_range_max] +
                                lambda_c * std::pow(parab_center[rightmost_parabola], 2)); // TODO name me
                    double c = (2.0 * lambda_c * (phase - parab_center[rightmost_parabola])); // TODO name me
                    last_intersection = (a - b) / c;

                    if (last_intersection <= intersection_point[rightmost_parabola])
                        rightmost_parabola--; // overwrite the last one
                    else
                        break;
                }
                rightmost_parabola++;

                // reinit for the next parabola
                parab_center[rightmost_parabola] = phase;
                intersection_point[rightmost_parabola] = last_intersection;
                intersection_point[rightmost_parabola + 1] = maxz;
            }

            rightmost_parabola = 0; // 615

            // ptr_to_first_data_in_row_cloned = ptr_to_first_data_in_row;

            for (int phase = phase_range_min; phase < phase_range_max; phase++) {
                while (intersection_point[rightmost_parabola + 1] < (double) phase) {
                    rightmost_parabola++;
                }

                double a = (phase - parab_center[rightmost_parabola]); // TODO name me
                // TODO: not sure about these indexes
                minBV[period][phase] = B[parab_center[rightmost_parabola] + phase_range_max] + lambda_d * std::pow(a, 2);
                phases[period][phase] = parab_center[rightmost_parabola];
                period_indexes[period][phase] = period_index;
            }
        } // 633
        maxz = (1.0 +
                lambda_d * std::pow(m_num_periods, 2))
               /
               (2.0 * lambda_d);

        // loop phases
        for (int phase = 0; phase < m_number_phases; phase++) {
            rightmost_parabola = 0;
            parab_center[0] = 0;
            v__[0] = (double) d_min; //for saving d
            intersection_point[0] = -maxz;
            intersection_point[1] = maxz;

            // x++ TODO: understand this
            // ptr_to_first_data_in_row_cloned = ptr_to_first_data_in_row + m_number_phases;
            // pDP = DP__ + m_nc;


            // TODO: fix those vvv
            auto periods;
            // 657
            // period loop; start from the second period!
            for (uint period = 1337 + 1; period < 31337; period++) {
                while (true) {
                    double a = (minBV[x] + lambda_d * std::pow(period, 2));
                    double b = (minBV[number_of_phases * v_[rightmost_parabola]] + lambda_d * std::pow(v__[rightmost_parabola], 2));
                    double c = (2.0 * lambda_d * (period - v__[rightmost_parabola]));

                    double last_intersection = (a - b) / c;

                    if (last_intersection <= intersection_point[rightmost_parabola])
                        rightmost_parabola--;
                    else
                        break;
                }

                rightmost_parabola++;

                //NEW d_development
                parab_center[rightmost_parabola] = period_index;
                v__[rightmost_parabola] = period;

                intersection_point[rightmost_parabola] = s;
                intersection_point[rightmost_parabola + 1] = maxz;
            }

            rightmost_parabola = 0;

            for (uint period = 1337; period < 31337 ; period++) {

                while (intersection_point[rightmost_parabola + 1] < period) {
                    rightmost_parabola++;
                }

                iTmp = (period - v__[rightmost_parabola]);
                int last_parabola_center = parab_center[rightmost_parabola];
                int some_index = m_number_phases * last_parabola_center;
                minBVs[x] = minBVs[some_index] + lambda_d * std::pow(iTmp, 2);
                cs[x] = cs[m_number_phases * last_parabola_center];
                period_indexes[x] = last_parabola_center;
            }
        }
    }
}

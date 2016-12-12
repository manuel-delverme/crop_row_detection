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
        // const std::vector<std::pair<int, int>> Xs,
        std::map<uint, std::vector<int>> Xs,
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
        for(std::pair<uint, std::vector<int>> const& item: Xs){
            uint period = item.first;
            std::vector<int> phases = item.second;
            for(int phase: phases) {
                std::pair<uint, int> x = std::make_pair(phase, period);

                energy = CrossCorrelation(image_row_num, x, positive_pulse_width, negative_pulse_width, window_width);
                row_energy_map[x] = energy;
                if (energy > best_energy) {
                    best_energy = energy;
                    best_pair = x;
                }
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

std::vector<tuple_type> CropRowDetector::find_best_parameters(const std::vector<std::map<tuple_type, double>> energy_map,
                                      std::map<period_type, std::vector<phase_type >> Xs) {
    uint image_height = (uint) energy_map.size();
    // std::vector<std::pair<int, int>> best_values;
    // std::vector<double> max_energy;
    // std::vector<std::map<std::pair<int,int>, double>> best_score_in_row(image_height);

    cv::Mat cost;

    // TODO to be defined with real variables
    float lambda_c = 0.5;
    float lambda_d = 0.5;

    // int m_num_periods = 31;
    // int m_number_phases = 31;

    // int m_min_d = 13;
    std::vector<phase_type> parab_center_phase(Xs.size());
    std::vector<phase_type> v_period(Xs.size());
    std::vector<phase_type> intersection_points(Xs.size());

    std::vector<std::map<tuple_type, double>> minBV = energy_map;
    // std::vector<std::map<tuple_type, double>> phases;

    for (uint row_number = 0; row_number < image_height; row_number++) {
        // max_energy[row_number] = std::max(energy.at<double>(row_number));
        int k;

        // int d_min = 8;
        int period_index;

        double left_term;
        double right_term;
        double normalizer;
        int last_intersection;
        // best_score_in_row[row_number] = 1.337; // this comes from template_matching, best of this row
        // double best_score_in_this_row = 1.337; // best_score_in_row[row_number];

        // TODO explain
        // normalizer = 1; // (2.0 * lambda_d * (period - v__[k])); // why??

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

        // for(std::vector<uint> phases: Xs){
        // for (uint period = 0; period < m_num_periods; period++) {
        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;
            k = 0;

            // set the first parabola
            uint phase_range_offset = (uint) std::floor(phases.size() / 2);
            uint phase_range_max = (uint) (phases.size() - phase_range_offset - 1);
            int phase_range_min = -phase_range_offset;

            // projection of parabola focus/vertex on the period
            v_period[0] = 0; // left bound

            intersection_points[0] = minz; // -inf
            intersection_points[1] = maxz; // +inf

            // for the second parabola until the last
            // for (int phase = phase_range_min + 1; phase < phase_range_max; phase++) // 593
            double fq;
            double q2;
            double fvk;
            double vk2;
            for (auto phase_iter=phases.begin() + 1; phase_iter != phases.end(); phase_iter++)
            {
                phase_type phase = *phase_iter;
                tuple_type x = std::make_pair(phase, period);
                assert((phase + phase_range_offset) > 0);
                uint q = (uint) (phase + phase_range_offset);

                while (true) {
                    fq = energy_map.at(row_number).at(x);
                    q2 = lambda_c * std::pow(q, 2);

                    tuple_type x_vk = std::make_pair(v_period[k] - (int) phase_range_offset, period);
                    fvk = energy_map.at(row_number).at(x_vk);

                    vk2 = lambda_c * std::pow(v_period[k], 2);
                    normalizer = 2.0 * lambda_c * (q - v_period[k]);

                    last_intersection = (int) (( (fq + q2) - (fvk + vk2) ) / normalizer);

                    if (last_intersection <= intersection_points[k]){
                        k--; // overwrite the last one
                    } else {
                        break;
                    }
                }
                k++;

                // reinit for the next parabola
                v_period[k] = q;
                intersection_points[k] = last_intersection;
                intersection_points[k + 1] = maxz;
            }

            k = 0; // 615

            double best_cost;
            double distance;
            for(phase_type phase: phases){
                uint q;
                assert(phase_range_offset + phase >= 0);
                q = (uint) (phase + phase_range_offset);

                while (intersection_points[k + 1] < q) {
                    k++;
                }
                tuple_type x = std::make_pair(phase, period);

                distance = (q - v_period[k]);

                tuple_type x_prime = std::make_pair(v_period[k] - phase_range_offset, period);

                best_cost = minBV.at(row_number)[x_prime] + lambda_c * std::pow(distance, 2);
                minBV.at(row_number).insert(std::make_pair(x, best_cost));
            }
        } // 633
        maxz = INT_MAX; // (1.0 + lambda_d * std::pow(m_num_periods, 2)) / (2.0 * lambda_d);

        int m_number_phases = 1337;
        // loop phases
        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;

            for (auto phase_iter=phases.begin() + 1; phase_iter != phases.end(); phase_iter++) {
                phase_type phase = *phase_iter;
                tuple_type x = std::make_pair(phase, period);

                k = 0;
                v_period[0] = 0;
                parab_center_phase[0] = Xs.begin()->first; //min_d
                intersection_points[0] = -maxz;
                intersection_points[1] = maxz;

                // period loop; start from the second period!

                tuple_type x_prime;
                while (true) {
                    left_term = minBV.at(row_number)[x] + lambda_d * std::pow(period, 2);

                    x_prime = std::make_pair(0, v_period[k]);
                    right_term = minBV.at(row_number)[x_prime] + lambda_d * std::pow(parab_center_phase[k], 2);

                    normalizer = (2.0 * lambda_d * (period - parab_center_phase[k]));

                    // s = () - (pointer_to_period_data[number_of_phases * v_period[k]].minBV + m_lambdad * (double)(parab_center_phase[k] * parab_center_phase[k]))) /
                    //     (2.0 * m_lambdad * (double)(d - parab_center_phase[k]));

                    last_intersection = (int) ((left_term - right_term) / normalizer);

                    if (last_intersection <= intersection_points[k])
                        k--;
                    else
                        break;
                }

                k++;

                //NEW d_development
                v_period[k] = phase;
                parab_center_phase[k] = period;

                intersection_points[k] = last_intersection;
                intersection_points[k + 1] = maxz;
            }
            k = 0;
            // reset phase
            for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
                period_type period = item.first;
                std::vector<phase_type> phases = item.second;

                for(int phase: phases) {

                    while (intersection_points[k + 1] < period) {
                        k++;
                    }

                    uint iTmp = period - v_period[k];

                    int last_parabola_center = v_period[k];
                    int some_index = m_number_phases * last_parabola_center;

                    int max_phases = 1337;
                    std::pair<int, int> x = std::make_pair(v_period[k] + max_phases, period);

                    std::pair<int, int> x_prime = std::make_pair(v_period[k] + max_phases, period);

                    // idk about x, x_prime
                    minBV.at(row_number)[x] = minBV.at(row_number)[x_prime] + lambda_d * std::pow(iTmp, 2);
                    std::pair<int, int> x_second = std::make_pair(phase, v_period[k]);
                    // _phases[row_number][x] = _phases[row_number][m_number_phases * last_parabola_center];
                    // _period_indexes[row_number][x] = v_period[k];
                }
            }
        }
    }
    std::vector<tuple_type> fake;
    return fake;
}

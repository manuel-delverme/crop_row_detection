//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <set>
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

std::vector<tuple_type> CropRowDetector::template_matching(
        std::vector<std::map<tuple_type, double>> & energy_map,
        const cv::Mat& Intensity,
        // const std::vector<std::pair<int, int>> Xs,
        std::map<period_type, std::vector<phase_type>> Xs,
        const double positive_pulse_width,
        const double negative_pulse_width,
        const int window_width // w
) {
    size_t image_height = (size_t) Intensity.size[0];

    double energy = 0;
    double best_energy = -1;

    tuple_type best_pair;
    std::vector<tuple_type> best_pairs(image_height);
    std::map<tuple_type, double> row_energy_map;

    for (uint image_row_num = 0; image_row_num < image_height; image_row_num++) {
        row_energy_map.clear();
        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs) {
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;
            for(const phase_type phase: phases) {
                tuple_type x = std::make_pair(phase, period);

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
double CropRowDetector::CrossCorrelation(int row_number, tuple_type template_var_param,
                                         double positive_pulse_width, double negative_pulse_width,
                                         int image_width){

    phase_type phase = template_var_param.first;
    period_type period = template_var_param.second;

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

inline double * CropRowDetector::distance_transform(std::vector<double> values, size_t length) {
    // TODO remove news

    // TODO to be defined with real variables
    // TODO readd labmdas
    float lambda_c = 0.5;
    float lambda_d = 0.5;

    double *d = new double[length];
    int *parabola_centers = new int[length];
    double *intersection_points = new double[length+1];
    int parabola_idx = 0;
    double intersection_point;

    parabola_centers[0] = 0;
    intersection_points[0] = -INFINITY;
    intersection_points[1] = +INFINITY;
    for (int q = 1; q <= length-1; q++) {
        intersection_point = ((values[q] + std::pow(q, 2)) - (values[parabola_centers[parabola_idx]] + std::pow(parabola_centers[parabola_idx], 2))) / (2 * q - 2 * parabola_centers[parabola_idx]);
        while (intersection_point <= intersection_points[parabola_idx]) {
            parabola_idx--;
            intersection_point  = ((values[q]+std::pow(q, 2))-(values[parabola_centers[parabola_idx]]+std::pow(parabola_centers[parabola_idx], 2)))/(2*q-2*parabola_centers[parabola_idx]);
        }
        parabola_idx++;
        parabola_centers[parabola_idx] = q;
        intersection_points[parabola_idx] = intersection_point;
        intersection_points[parabola_idx+1] = +INFINITY;
    }

    parabola_idx = 0;
    for (int q = 0; q <= length-1; q++) {
        while (intersection_points[parabola_idx+1] < q){
            parabola_idx++;
        }
        d[q] = std::pow(q-parabola_centers[parabola_idx], 2) + values[parabola_centers[parabola_idx]];
    }

    delete [] parabola_centers;
    delete [] intersection_points;
    return d;
}

std::vector<tuple_type> CropRowDetector::find_best_parameters(std::vector<std::map<tuple_type, double>> energy_map,
                                      const std::map<period_type, std::vector<phase_type >>& Xs) {

    // TODO:
    // should initialize all the unused phases (most of them for low periods)
    // to +inf and ignore them if required
    size_t image_height = (size_t) energy_map.size();
    size_t num_periods = Xs.size();

    std::set<phase_type> all_the_phases;
    std::vector<tuple_type> best_nodes(image_height);

    for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
        std::vector<phase_type> phases = item.second;
        for(const phase_type phase: phases) {
            all_the_phases.insert(phase);
        }
    }
    size_t num_phases = all_the_phases.size();

    double *distances;
    std::vector<double> f(std::max(num_periods, num_phases));

    for (size_t row_number = 0; row_number < image_height; row_number++) {
        std::cout << "working on row " << row_number << std::endl;

        for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
            const period_type period = item.first;
            std::vector<phase_type> phases = item.second;

            // std::cout << "[F] period: " << period << std::endl;

            phase_type phase_range_offset; // TODO use negative phases
            phase_range_offset = -*std::min_element(phases.begin(), phases.end());
            for (phase_type phase: phases){
                // std::cout << "\t[F] phase: " << phase << std::endl;
                const tuple_type x = std::make_pair(phase, period);
                f[phase + phase_range_offset] = energy_map.at(row_number).at(x);
            }
            distances = distance_transform(f, phases.size());
            for (phase_type phase: phases){
                // const phase_type phase = *phase_iter;
                // std::cout << "\t[B] phase: " << phase << std::endl;
                const tuple_type x = std::make_pair(phase, period);
                // if(energy_map.at(row_number).find(x)->second != distances[phase + phase_range_offset]){
                //     std::cout << "\t[B] replacing: " << energy_map.at(row_number).find(x)->second << " with " << distances[phase + phase_range_offset] << std::endl;
                // }

                energy_map.at(row_number).find(x)->second = distances[phase + phase_range_offset];
                // std::cout << "\t[*] is now: " << energy_map.at(row_number).find(x)->second << std::endl;
            }
        } // 633
        std::cout << "done; working on periods" << std::endl;

        // std::vector<period_type> period_intersection_points(Xs.size());

        //TODO think of a better structure
        for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
            period_type period = item.first;
            m_period_map.push_back(period);
        }

        for (phase_type phase: all_the_phases){
            std::cout << std::endl << "phase " << phase << std::endl;
            auto periods = periods_of(Xs, phase);
            for (period_type period: periods) {
                // std::cout << "\t[F] period: " << period << std::endl;
                // period_type period = item.first;
                // std::vector<phase_type> phases = item.second;
                tuple_type x = std::make_pair(phase, period);
                f.at(index_of_period(period)) = energy_map.at(row_number).at(x);
            }
            distances = distance_transform(f, num_phases);
            for (period_type period: periods_of(Xs, phase)) {
                // std::cout << "\t[B] period: " << period << std::endl;
                tuple_type x = std::make_pair(phase, period);
                auto val_ptr = energy_map.at(row_number).find(x);
                // if(val_ptr->second != distances[index_of_period(period)]){
                //     std::cout << "\t\t[B] replacing: " << val_ptr->second << " with "
                //               << distances[index_of_period(period)] << std::endl;
                // }
                energy_map.at(row_number).find(x)->second = distances[index_of_period(period)];
                // std::cout << "\t[*] is now: " << energy_map.at(row_number).find(x)->second << std::endl;
            }
        }
    }
    size_t row_number = image_height - 1;

    tuple_type best_node;
    double best_node_value = INFINITY;

    for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
        period_type period = item.first;
        std::vector<phase_type> phases = item.second;
        for(phase_type phase: phases) {
            tuple_type x = std::make_pair(phase, period);

            double x_energy = energy_map.at(row_number).at(x);
            if(x_energy < best_node_value)
            {
                best_node = x;
            }
        }
    }
    best_nodes.at(row_number) = best_node;

    /*
     * In the second stage, represented by lines 18 and 19, the optimal
     * vector x is selected for each image row, starting from the last one.
     * In line 18, the vector x, for which the value Bh?1 x ðÞ is minimal,
     * is selected as the optimal interpretation of the last image row.
     *
     * ##########
     * In line 19, for each preceding image row v, vector xv,
     * which is optimal for xvþ1 computed in the previous iteration,
     * is selected as the optimal interpretation of that image row.
     * This vector corresponds to the vector Tv(xvþ1) stored during the first stage.
     * At the end of the second stage, optimal vectors xv are obtained for all image rows.
     * The obtained sequence of vectors x0, x1, …, xh?1
     * represents the optimal crop model for a given image according to the proposed criterion.
     * */
    double tuple_data;

    double x_energy;

    for(row_number = image_height - 2; row_number >= 0 && row_number < image_height; row_number--) {
        best_node_value = INFINITY;
        for(auto const& item: energy_map.at(row_number)) {
            tuple_type x = item.first;
            x_energy = item.second;
            if(x_energy < best_node_value)
            {
                best_node = x;
                best_node_value = x_energy;
            }
        }
        best_nodes.at(row_number) = best_node;
    }
    return best_nodes;
}

size_t CropRowDetector::index_of_period(period_type period) {
    for(size_t i = 0; i < m_period_map.size(); i++){
        if(m_period_map[i] == period){
            return i;
        }
    }
    throw std::exception();
}

std::vector<period_type>
CropRowDetector::periods_of(const std::map<period_type, std::vector<phase_type>> &Xs, const phase_type phase) {
    std::vector<period_type> periods;
    // TODO NOOOOPE, lookup is o(N), use a set? or hashmap?, actually it can be calculated with f(period/2)
    for(std::pair<period_type, std::vector<phase_type>> const& item: Xs) {
        period_type period = item.first;
        std::vector<phase_type> phases = item.second;
        if(std::find(phases.begin(), phases.end(), phase) != phases.end()) {
            periods.push_back(period);
        }
    }
    return periods;
}

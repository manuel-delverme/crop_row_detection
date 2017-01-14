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

std::vector<tuple_type> CropRowDetector::find_best_parameters(std::vector<std::map<tuple_type, double>> energy_map,
                                      const std::map<period_type, std::vector<phase_type >>& Xs) {

    // TODO:
    // should initialize all the unused phases (most of them for low periods)
    // to +inf and ignore them if required
    size_t image_height = (size_t) energy_map.size();
    size_t num_periods = Xs.size();


    // TODO: remove the following
    std::vector<std::map<tuple_type, double>> B(energy_map);
    std::vector<std::map<tuple_type, double>> D(energy_map);
    std::vector<std::map<tuple_type, double>> minBV(energy_map);
    std::vector<std::map<tuple_type, phase_type>> c(image_height);
    std::vector<std::map<tuple_type, size_t>> d(image_height);
    // up to here

    std::set<phase_type> all_the_phases;
    std::vector<tuple_type> best_nodes(image_height);

    for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
        std::vector<phase_type> phases = item.second;
        for(const phase_type phase: phases) {
            all_the_phases.insert(phase);
        }
    }
    size_t num_phases = all_the_phases.size();


    std::vector<std::map<phase_type, phase_type>> precedent_best_phase(image_height);
    std::vector<std::map<period_type, period_type>> precedent_best_period(image_height);
    // double *distances;

    // std::vector<double> f(std::max(num_periods, num_phases));
    auto comparer = [] (const auto & p1, const auto & p2) { return p1.second < p2.second;};
    double Dnrm;
    // phase_type phase;
    // period_type period;
    std::vector<phase_type> phases;

    for (size_t row_number = 0; row_number < image_height; row_number++) {
        std::cout << "working on row " << row_number << std::endl;
        Dnrm = std::max_element(std::begin(energy_map.at(row_number)), std::end(energy_map.at(row_number)), comparer)->second;

        for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
            period_type period = item.first;

            for (phase_type phase: item.second) {

                const tuple_type x = std::make_pair(phase, period);
                // double b_val;

                if(Dnrm >= m_f_low)
                {
                    // b_val = 1.0 - energy_map.at(row_number).at(x) / Dnrm;
                    B.at(row_number).at(x) = 1.0 - energy_map.at(row_number).at(x) / Dnrm;

                    if(B.at(row_number).at(x) > m_maxD){
                        B.at(row_number).at(x) = m_maxD;
                    }
                } else {
                    B.at(row_number).at(x) = m_maxD;
                }

                if(row_number > 0){
                    B.at(row_number).at(x) += minBV.at(row_number - 1).at(x);
                }
                // energy_map.at(row_number).at(x) = b_val;
                // TODO: ENERGYMAP == D
            }
        }
        phase_type phase_range_offset; // TODO use negative phases

        if(row_number < image_height - 1) {
            for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
                period_type period = item.first;
                phases = item.second;
                // phase_range_offset = -phases.at(0);

                size_t length = phases.size();
                phase_type *parabola_centers = new phase_type[length];
                double *intersection_points = new double[length+1];
                int parabola_idx = 0;
                double intersection_point;

                parabola_centers[0] = phases.at(0);
                intersection_points[0] = -INFINITY;
                intersection_points[1] = +INFINITY;

                tuple_type vk;
                for (const phase_type phase: phases) {
                    const tuple_type x = std::make_pair(phase, period);
                    vk = std::make_pair(parabola_centers[parabola_idx], period);
                    double num_phase = B.at(row_number).at(x) + m_lambda_c * (double)(phase*phase);

                    while(true){
                        intersection_point = num_phase - B.at(row_number).at(vk) + m_lambda_c * std::pow(parabola_centers[parabola_idx], 2);
                        intersection_point /= 2.0 * m_lambda_c * (phase - parabola_centers[parabola_idx]);
                        if(intersection_point <= intersection_points[parabola_idx]) {
                            parabola_idx--;
                        } else {
                            break;
                        }
                    }
                    parabola_idx++;
                    parabola_centers[parabola_idx] = phase;
                    intersection_points[parabola_idx] = intersection_point;
                    intersection_points[parabola_idx+1] = +INFINITY;
                }

                parabola_idx = 0;
                for (phase_type phase: phases) {
                    const tuple_type x = std::make_pair(phase, period);
                    while (intersection_points[parabola_idx + 1] < (double) phase) {
                        parabola_idx++;
                    }
                    vk = std::make_pair(parabola_centers[parabola_idx], period);
                    double phase_dist = phase - parabola_centers[parabola_idx];
                    minBV.at(row_number).at(x) = B.at(row_number).at(vk) + m_lambda_c * phase_dist * phase_dist;
                    c.at(row_number).at(x) = parabola_centers[parabola_idx];
                }
            }
            std::cout << "done; working on periods" << std::endl;

            //TODO think of a better structure to iterate on
            for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
                const period_type period = item.first;
                m_period_map.push_back(period);
            }

            for (phase_type phase: all_the_phases) {
                std::cout << std::endl << "phase " << phase << std::endl;
                auto periods = periods_of(Xs, phase);
                size_t length = periods.size();
                period_type *parabola_centers = new period_type[length];
                double *intersection_points = new double[length+1];
                int parabola_idx = 0;
                double intersection_point;

                parabola_centers[0] = 0;
                intersection_points[0] = -INFINITY;
                intersection_points[1] = +INFINITY;

                tuple_type vk;
                for (const period_type period: periods) {
                    const tuple_type x = std::make_pair(phase, period);
                    vk = std::make_pair(parabola_centers[parabola_idx], period);
                    double num_period = minBV.at(row_number).at(x) + m_lambda_d * (double) (period * period);
                    while (true) {
                        intersection_point = num_period - minBV.at(row_number).at(vk) +
                                             m_lambda_d * std::pow(parabola_centers[parabola_idx], 2);
                        intersection_point /= 2.0 * m_lambda_d * (phase - parabola_centers[parabola_idx]);
                        if (intersection_point <= intersection_points[parabola_idx]) {
                            parabola_idx--;
                        } else {
                            break;
                        }
                    }
                    parabola_idx++;
                    parabola_centers[parabola_idx] = phase;
                    intersection_points[parabola_idx] = intersection_point;
                    intersection_points[parabola_idx + 1] = +INFINITY;
                }
                parabola_idx = 0;
                for (const period_type period: periods) {
                    const tuple_type x = std::make_pair(phase, period);
                    while (intersection_points[parabola_idx + 1] < (double) period) {
                        parabola_idx++;
                    }
                    vk = std::make_pair(parabola_centers[parabola_idx], period);
                    double period_dist = period - parabola_centers[parabola_idx];
                    minBV.at(row_number).at(x) = B.at(row_number).at(vk) + m_lambda_d * period_dist * period_dist;
                    c.at(row_number).at(x) = c.at(row_number).at(vk);
                    d.at(row_number).at(x) = index_of_period(period);
                }
            }
        }
    }
    size_t row_number = image_height - 1;

    tuple_type best_node;
    double best_node_B = INFINITY;

    for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
        const period_type period = item.first;
        phases = item.second;
        for(phase_type phase: phases) {
            const tuple_type x = std::make_pair(phase, period);

            if(B.at(row_number).at(x) < best_node_B)
            {
                best_node = x;
                best_node_B = B.at(row_number).at(x);
            }
        }
    }
    best_nodes.at(row_number) = best_node;

    phase_type best_phase = best_node.first;
    period_type best_period = best_node.second;

    while(row_number-- > 0) {
        best_phase = precedent_best_phase.at(row_number).at(best_phase);
        best_period = precedent_best_period.at(row_number).at(best_period);
        best_nodes.at(row_number) = std::make_pair(best_phase, best_period);

        tuple_type node = best_nodes.at(row_number);
        auto energies = energy_map.at(row_number);
        double x = energies.at(node);
        std::cout << "energy " << x << std::endl;
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

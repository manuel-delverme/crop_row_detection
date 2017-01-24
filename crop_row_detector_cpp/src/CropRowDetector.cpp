//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <unordered_map>
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
    m_period_scale_factor = .125;
    m_half_width = intensity_map.cols/2;

}

std::vector<old_tuple_type> CropRowDetector::template_matching(
        std::vector<std::map<old_tuple_type, double>> & energy_map,
        const cv::Mat& Intensity,
        // const std::vector<std::pair<int, int>> Xs,
        std::map<period_type, std::vector<phase_type>> Xs,
        const double positive_pulse_width,
        const double negative_pulse_width,
        const size_t window_width // w
) {
    size_t image_height = (size_t) Intensity.size[0];

    double energy = 0;
    double best_energy = -1;

    old_tuple_type best_pair;
    std::vector<old_tuple_type> best_pairs(image_height);
    std::map<old_tuple_type, double> row_energy_map;

    for (size_t image_row_num = 0; image_row_num < image_height; image_row_num++) {
        row_energy_map.clear();
        // size_t period_idx = 0;
        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs) {
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;
            for(const phase_type phase: phases) {
                old_tuple_type x = std::make_pair(phase, period);

                energy = CrossCorrelation((int) image_row_num, x, positive_pulse_width, negative_pulse_width, window_width);
                row_energy_map[x] = energy;
                if (energy > best_energy) {
                    best_energy = energy;
                    best_pair = x;
                }
            }
        }
        energy_map.at(image_row_num) = row_energy_map;
        // TODO: extract best energy value Dnrm for later processing
        best_pairs[image_row_num] = best_pair;
        best_energy = -1;
    }
    return best_pairs;
}
double CropRowDetector::CrossCorrelation(int row_number, old_tuple_type template_var_param,
                                         double positive_pulse_width, double negative_pulse_width,
                                         size_t image_width){

    phase_type phase = template_var_param.first;
    period_type period = template_var_param.second;

    int negative_pulse_end, negative_pulse_start, positive_pulse_end, positive_pulse_start, positive_pixels, negative_pixels;
    double positive_correlation_value, negative_correlation_value;


    // Calcolo quantità necesarrie a CrossCorrelation
    double scale = period*m_period_scale_factor;

    int a = DOUBLE2INT(scale*positive_pulse_width);
    int b = DOUBLE2INT(scale*negative_pulse_width);

    double halfb = .5*b;
    double halfa = .5*a;
    double halfd = .5*period;

    int kStart = (int)floor((-((int) image_width / 2 + phase) + halfa) / period);    // offset prima onda
    int kEnd = (int)floor((((int) image_width / 2 - phase) + halfa) / period);   // offset ultima onda prima della fine dell'immagine


    // Calcolo centro dell'onda quadra positiva e negativa
    double distance_positive_negative_pulse_center =  halfd - halfb;
    double positive_pulse_center = (double)phase + (double)kStart * (double)period + m_half_width;


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

std::vector<old_tuple_type> CropRowDetector::find_best_parameters(std::vector<std::map<old_tuple_type, double>>& energy_map,
                                      const std::map<period_type, std::vector<phase_type >>& Xs) {

    // TODO:
    // should initialize all the unused phases (most of them for low periods)
    // to +inf and ignore them if required
    size_t image_height = (size_t) energy_map.size();

    // std::vector<std::map<tuple_type, data_type>> dataset(image_height);

    // TODO: reserve space for hashmaps
    std::vector<std::unordered_map<tuple_type, data_type>> dataset(image_height);

    std::vector<phase_type> all_the_phases = Xs.rbegin()->second;
    std::vector<old_tuple_type> best_nodes(image_height);

    auto comparer = [] (const auto & p1, const auto & p2) { return p1.second < p2.second;};
    double Dnrm;
    std::vector<phase_type> phases;

    int parabola_idx;
    // size_t length = (size_t) Xs.rbegin()->first;
    size_t length = (size_t) std::max(Xs.size(), (size_t) Xs.rbegin()->first);

    // phase_type* parabola_centers_phase = new phase_type[length];
    period_type parabola_centers_period[length];
    size_t* parabola_centers_period_idx = new size_t[length];
    double *intersection_points = new double[length+1];
    std::vector<double> intersection_points_phase(length+1);
    std::vector<phase_type> parabola_centers_phase(length);

    double numerator;
    double b_val;

    std::vector<period_type> periods((size_t) m_nd);
    period_type period = m_mind;
    for (size_t period_idx = 0; period_idx < m_nd; period_idx++, period *= m_dstep) {
        periods[period_idx] = period;
    }

    for (size_t row_number = 0; row_number < image_height; row_number++) {
        // std::cout << "working on row " << row_number << std::endl;
        //TODO: take it from template_matching
        Dnrm = std::max_element(std::begin(energy_map.at(row_number)), std::end(energy_map.at(row_number)), comparer)->second;

        size_t period_idx = 0;
        for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
            period = item.first;
            if(period != periods[period_idx]){
                // std::cout << "replacing " << periods[period_idx] << " with " << period << std::endl;
                periods[period_idx] = period; // HACK FOR csv generated
            }
            for (const phase_type phase: item.second) {
                const old_tuple_type x_energy = std::make_pair(phase, period);
                const tuple_type x = std::make_pair(phase, period_idx);

                auto D = energy_map.at(row_number).at(x_energy);
                if(Dnrm >= m_f_low)
                {
                    b_val = 1.0 - D / Dnrm;
                    // B.at(row_number).at(x) = 1.0 - energy_map.at(row_number).at(x) / Dnrm;

                    if(b_val > m_maxD){
                        b_val = m_maxD;
                    }
                } else {
                    b_val = m_maxD;
                }

                if(row_number > 0){
                    b_val += dataset.at(row_number - 1).at(x).minBV;
                }
                data_type p;
                p.B = b_val;
                p.c = phase;
                p.d = period_idx;
                p.minBV = INT_MAX;
                dataset.at(row_number)[x] = p;
            }
            period_idx++;
        }
        if(row_number < image_height - 1) {
            // #pragma omp parallel for default(none) private(period, parabola_idx, numerator) shared(periods, dataset, row_number, length)
            for (size_t period_idx = 0; period_idx < m_nd; period_idx++) {
                period = periods[period_idx];
                // intersection_points = new double[length+1];
                double intersection_point;

                phase_type half_band = (phase_type) std::floor(0.5 * period);
                parabola_idx = 0;

                parabola_centers_phase[0] = -half_band;
                intersection_points_phase[0] = -INFINITY;
                intersection_points_phase[1] = +INFINITY;

                for (phase_type phase = -half_band + 1; phase < half_band; phase++) {
                    const data_type x = dataset.at(row_number).at(std::make_pair(phase, period_idx));
                    numerator = x.B + m_lambda_c * (double) (phase * phase);

                    while (true) {
                        auto vkx = std::make_pair(parabola_centers_phase[parabola_idx], period_idx);
                        const auto vk = dataset.at(row_number).at(vkx);
                        intersection_point =
                                numerator - (vk.B + m_lambda_c * std::pow(parabola_centers_phase[parabola_idx], 2));
                        intersection_point /= 2.0 * m_lambda_c * (phase - parabola_centers_phase[parabola_idx]);

                        if (intersection_point <= intersection_points_phase[parabola_idx]) {
                            parabola_idx--;
                        } else {
                            break;
                        }
                    }
                    parabola_idx++;
                    parabola_centers_phase[parabola_idx] = phase;
                    intersection_points_phase[parabola_idx] = intersection_point;
                    intersection_points_phase[parabola_idx + 1] = +INFINITY;
                }

                parabola_idx = 0;
                for (phase_type phase = -half_band; phase < half_band; phase++) {
                    auto x = &dataset.at(row_number).find(std::make_pair(phase, period_idx))->second;
                    while (intersection_points_phase.at(parabola_idx + 1) < (double) phase) {
                        parabola_idx++;
                    }
                    const auto vk = dataset.at(row_number).at(
                            std::make_pair(parabola_centers_phase[parabola_idx], period_idx));
                    const double phase_dist = phase - parabola_centers_phase[parabola_idx];
                    x->minBV = vk.B + m_lambda_c * phase_dist * phase_dist;
                    x->c = parabola_centers_phase[parabola_idx];
                    x->d = period_idx;
                }

                phase_type real_phase;
                for (phase_type phase = -m_nc/2; phase < m_nc/2; phase++) {
                    if (phase >= -half_band && phase < half_band) {
                        continue;
                    }
                    real_phase = ((phase + half_band) % (int) std::floor(period)) - half_band;
                    if(real_phase == -half_band)
                        parabola_idx = 0;

                    auto x = &dataset.at(row_number).find(std::make_pair(real_phase, period_idx))->second;

                    while (intersection_points_phase.at((size_t) (parabola_idx + 1)) < (double) real_phase) {
                        parabola_idx++;
                    }
                    const auto vk = dataset.at(row_number).at(std::make_pair(parabola_centers_phase[parabola_idx], period_idx));
                    const double phase_dist = real_phase - parabola_centers_phase[parabola_idx];
                    x->minBV = vk.B + m_lambda_c * phase_dist * phase_dist;
                    x->c = parabola_centers_phase[parabola_idx];
                    x->d = period_idx;
                }
            }

            // ----------------------------------------------------------------------------------------------------
#if DEBUG
            std::fstream outfile; outfile.open("/tmp/results_mio_phase", std::fstream::out);
            for (size_t period_idx = 0; period_idx < m_nd; ++period_idx) {
                period_type period = periods[period_idx];
                phase_type half_band = (phase_type) std::floor(0.5 * period);

                for (phase_type phase = -129; phase < -half_band + 1; phase++)
                    outfile << row_number << "," << period_idx << "," << phase << ",COCKS,COCKS" << std::endl;

                for (phase_type phase = -half_band + 1; phase < half_band; phase++) {
                    auto x = &dataset.at(row_number).find(std::make_pair(phase, period_idx))->second;
                    outfile << row_number << "," << period_idx << "," << phase << "," << x->B << "," << x->minBV << std::endl;
                }
                for (phase_type phase = half_band; phase < 129; phase++)
                    outfile << row_number << "," << period_idx << "," << phase << ",COCKS,COCKS" << std::endl;

            }
            outfile.close();
#endif
            // ----------------------------------------------------------------------------------------------------

            // std::cout << "done; working on periods" << std::endl;

            for (phase_type phase = -(m_nc / 2); phase < m_nc / 2; phase++) {
                // std::cout << "phase " << phase << std::endl;
                double intersection_point;
                size_t p_min_idx = period_min(phase, periods);

                parabola_idx = 0;
                parabola_centers_period[0] = periods[p_min_idx];
                parabola_centers_period_idx[0] = p_min_idx;
                intersection_points[0] = -INFINITY;
                intersection_points[1] = +INFINITY;

                for (period_idx = p_min_idx+1; period_idx < m_nd; period_idx++) {
                    period = periods[period_idx];
                    auto const x = dataset.at(row_number).at(std::make_pair(phase, period_idx));
                    numerator = x.minBV + m_lambda_d * (double) (period * period);

                    while (true) {
                        const auto vk = dataset.at(row_number).at(std::make_pair(phase, parabola_centers_period_idx[parabola_idx]));
                        intersection_point = numerator - (vk.minBV + m_lambda_d * std::pow(parabola_centers_period[parabola_idx], 2));
                        intersection_point /= 2.0 * m_lambda_d * (period - parabola_centers_period[parabola_idx]);

                        if (intersection_point <= intersection_points[parabola_idx]) {
                            parabola_idx--;
                        } else {
                            break;
                        }
                    }
                    parabola_idx++;
                    parabola_centers_period[parabola_idx] = period;
                    parabola_centers_period_idx[parabola_idx] = period_idx;
                    intersection_points[parabola_idx] = intersection_point;
                    intersection_points[parabola_idx + 1] = +INFINITY;
                }
                parabola_idx = 0;
                for (period_idx = p_min_idx; period_idx < m_nd; period_idx++) {
                    period = periods[period_idx];

                    while (intersection_points[parabola_idx + 1] < (double) period) {
                        parabola_idx++;
                    }
                    const auto vk = dataset.at(row_number).at(std::make_pair(phase, parabola_centers_period_idx[parabola_idx]));
                    const double period_dist = period - parabola_centers_period[parabola_idx];

                    auto x = &dataset.at(row_number).find(std::make_pair(phase, period_idx))->second;
                    x->minBV = vk.B + m_lambda_d * period_dist * period_dist;
                    x->c = vk.c;
                    x->d = parabola_centers_period_idx[parabola_idx];
                }
            }
        }
    }
    size_t row_number = image_height - 1;

    data_type best_node, node;
    best_node.B = dataset.at(row_number).at(std::make_pair(0, 0)).B;

    size_t period_idx = 0;
    for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
        // const period_type period = item.first;
        phases = item.second;
        for(phase_type phase: phases) {
            node = dataset.at(row_number).at(std::make_pair(phase, period_idx));
            if(node.B < best_node.B)
            {
                best_node = node;
            }
        }
        period_idx++;
    }
    best_nodes.at(row_number) = std::make_pair(best_node.c, periods[best_node.d]);

    phase_type best_phase; // = best_node.first;
    period_type best_period; //  = best_node.second;
    size_t best_period_idx;

    while(row_number-- > 0) {
        best_node = dataset.at(row_number).at(std::make_pair(best_node.c, best_node.d));
        best_nodes.at(row_number) = std::make_pair(best_node.c, periods[best_node.d]);

        // double x = dataset.at(row_number).at(best_node).minBV;
        std::cout << "row: " << row_number - 1 << " phase: " << best_node.c << " period:" << best_node.d << std::endl;
    }
    // delete[] intersection_points;
    // delete[] parabola_centers_period;
    // delete[] parabola_centers_period_idx;
    return best_nodes;
}

size_t CropRowDetector::period_min(const phase_type phase, std::vector<period_type> periods) {
    period_type period_min;
    if(phase < 0){
        period_min = (2 * -phase);
    } else {
        period_min = (2 * (phase + 1));
    }
    size_t period_idx;
    // for (period = m_mind; period < period_min; period *= m_dstep);
    for (period_idx = 0; periods[period_idx] < period_min; period_idx++);
    return period_idx;
}

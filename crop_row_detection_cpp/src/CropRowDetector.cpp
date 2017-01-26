//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "CropRowDetector.h"

// MACRO per fixing del biased rounding
#define DOUBLE2INT(x)    (int)floor(x + 0.5)

namespace crd_cpp {
    CropRowDetector::CropRowDetector() {}

    void CropRowDetector::load(cv::Mat const &intensity_map) {
        cv::Mat intensity_map_64f;
        intensity_map.convertTo(intensity_map_64f, CV_64F);
        m_integral_image = cv::Mat::zeros(intensity_map.size(), CV_64F);

        for (int row = 0; row < intensity_map.rows; row++) {
            for (int column = 1; column < intensity_map.cols; column++) {
                m_integral_image.at<double>(row, column) =
                        (double) intensity_map.at<uchar>(row, column) + m_integral_image.at<double>(row, column - 1);
            }
        }
        m_dataset_ptr = new data_type[intensity_map.rows * m_nd * m_nc];
        m_period_scale_factor = .125;
        m_half_width = intensity_map.cols / 2;

    }

    std::vector<old_tuple_type> CropRowDetector::template_matching(
            std::vector<std::map<old_tuple_type, double>> &energy_map,
            const cv::Mat &Intensity,
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
            for (std::pair<period_type, std::vector<phase_type>> const &item: Xs) {
                period_type period = item.first;
                std::vector<phase_type> phases = item.second;
                for (const phase_type phase: phases) {
                    old_tuple_type x = std::make_pair(phase, period);

                    energy = CrossCorrelation((int) image_row_num, x, positive_pulse_width, negative_pulse_width,
                                              window_width);
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
                                             size_t image_width) {

        phase_type phase = template_var_param.first;
        period_type period = template_var_param.second;

        int negative_pulse_end, negative_pulse_start, positive_pulse_end, positive_pulse_start, positive_pixels, negative_pixels;
        double positive_correlation_value, negative_correlation_value;


        // Calcolo quantità necesarrie a CrossCorrelation
        double scale = period * m_period_scale_factor;

        int a = DOUBLE2INT(scale * positive_pulse_width);
        int b = DOUBLE2INT(scale * negative_pulse_width);

        double halfb = .5 * b;
        double halfa = .5 * a;
        double halfd = .5 * period;

        int kStart = (int) floor((-((int) image_width / 2 + phase) + halfa) / period);    // offset prima onda
        int kEnd = (int) floor(
                (((int) image_width / 2 - phase) + halfa) /
                period);   // offset ultima onda prima della fine dell'immagine


        // Calcolo centro dell'onda quadra positiva e negativa
        double distance_positive_negative_pulse_center = halfd - halfb;
        double positive_pulse_center = (double) phase + (double) kStart * (double) period + m_half_width;


        // Calcolo per Onda ad inizio immagine, non calcolabile con la classica routine

        positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);
        positive_pulse_end = positive_pulse_start + a - 1;

        negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
        negative_pulse_end = negative_pulse_start + b - 1;

        if (positive_pulse_end >= 0) {
            positive_correlation_value = cumulative_sum(row_number, positive_pulse_end);
            positive_pixels = positive_pulse_end;
        } else {
            positive_correlation_value = 0;
            positive_pixels = 0;
        }


        if (negative_pulse_start < 0)
            negative_pulse_start = 0;

        if (negative_pulse_end >= 0) {
            negative_correlation_value = cumulative_sum(row_number, negative_pulse_end)
                                         - cumulative_sum(row_number,
                                                          negative_pulse_start); //tolto  -cumulative_sum(row_number, negative_pulse_start-1);

            negative_pixels = negative_pulse_end - negative_pulse_start + 1;
        } else {
            negative_correlation_value = 0;
            negative_pixels = 0;
        }


        positive_pulse_center += period;

        for (int k = kStart + 1; k < kEnd; k++, positive_pulse_center += period) {


            positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);
            positive_pulse_end = positive_pulse_start + a - 1;

            positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                          - cumulative_sum(row_number, positive_pulse_start - 1);

            positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

            negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
            negative_pulse_end = negative_pulse_start + b - 1;

            negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                          - cumulative_sum(row_number, negative_pulse_start - 1);

            negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
        }

        positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);

        positive_pulse_end = positive_pulse_start + a - 1;

        if (positive_pulse_end >= image_width)
            positive_pulse_end = image_width - 1;

        positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                      - cumulative_sum(row_number, positive_pulse_start - 1);

        positive_pixels += (positive_pulse_end - positive_pulse_start + 1);


        negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
        if (negative_pulse_start < image_width) {
            negative_pulse_end = negative_pulse_start + b - 1;

            if (negative_pulse_end >= image_width)
                negative_pulse_end = (int) (image_width - 1);

            negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                          - cumulative_sum(row_number, negative_pulse_start - 1);

            negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
        }

        return (negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) /
               (double) (positive_pixels * negative_pixels);
    }


    double CropRowDetector::cumulative_sum(int v, int start) {
        // TODO turn in start-end
        return m_integral_image.at<double>(v, start);
    }

    std::vector<old_tuple_type>
    CropRowDetector::find_best_parameters(std::vector<std::map<old_tuple_type, double>> &energy_map) {
        // CALLGRIND_START_INSTRUMENTATION;

        size_t image_height = (size_t) energy_map.size();
        std::vector<old_tuple_type> best_nodes(image_height);

        auto comparer = [](const auto &p1, const auto &p2) { return p1.second < p2.second; };

        size_t length = (size_t) std::max(m_nd, m_nc);

        phase_type parabola_center[length];
        energy_type intersection_points[length + 1];

        energy_type numerator;
        energy_type b_val;

        period_type periods[m_nd];
        period_type period = m_mind;
        // TODO: interleave data struct

        data_type *dataset_row_ptr = m_dataset_ptr;
        data_type *dataset_tuple_ptr;
        data_type *dataset_period_ptr;

        int row_size = m_nc * m_nd;

        energy_type intersection_point;
        period_idx_type period_idx;
        phase_type phase;
        energy_type Dnrm;
        const energy_type maxz = (const energy_type) ((1.0 + (m_lambda_c + m_lambda_d) * m_nd * m_nd) /
                                                      (m_lambda_c * m_lambda_d));
        int parabola_idx;
        const phase_type first_phase = (const phase_type) (-m_nc / 2);

        for (period_idx = 0; period_idx < m_nd; period_idx++, period *= m_dstep) {
            periods[period_idx] = period;
        }

        for (size_t row_number = 0; row_number < image_height; row_number++, dataset_row_ptr += row_size) {
            std::cout << row_number << std::endl;

            auto row_max = std::max_element(std::begin(energy_map.at(row_number)), std::end(energy_map.at(row_number)),
                                            comparer); //TODO: take it from template_matching
            Dnrm = (energy_type) row_max->second;
            dataset_period_ptr = dataset_row_ptr;        // dataset_period_ptr - ptr. to the first data in a block row

            for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
                dataset_tuple_ptr = dataset_period_ptr;
                const period_type period_ = periods[period_idx];

                for (phase = first_phase; phase < -first_phase; phase++, dataset_tuple_ptr++) {
                    const phase_type real_phase = get_real_phase(phase, period_);
                    const old_tuple_type x_energy = std::make_pair(real_phase, period_);

                    const energy_type D = (energy_type) energy_map.at(row_number).at(x_energy);
                    if (Dnrm >= 1.0) {
                        b_val = (energy_type) (1.0 - (D / Dnrm));

                        if (b_val > m_maxD) {
                            b_val = m_maxD;
                        }
                    } else {
                        b_val = m_maxD;
                    }
                    if (row_number > 0) {
                        b_val += (dataset_tuple_ptr - row_size)->tuple_value;
                    }
                    dataset_tuple_ptr->tuple_value = b_val;
                }
            }
            if (row_number < image_height - 1) {
                dataset_period_ptr = dataset_row_ptr;
                // #pragma omp parallel for default(none) private(period_idx, parabola_idx, numerator, dataset_period_ptr, dataset_tuple_ptr) shared(periods, row_number)
                for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
                    parabola_idx = 0;
                    parabola_center[0] = first_phase;
                    intersection_points[0] = -maxz;
                    intersection_points[1] = +maxz;
                    dataset_tuple_ptr = dataset_period_ptr + 1;

                    for (phase = first_phase + (phase_type) 1; phase < -first_phase; phase++, dataset_tuple_ptr++) {
                        numerator = dataset_tuple_ptr->tuple_value + m_lambda_c * phase * phase;
                        while (true) {
                            intersection_point = numerator - dataset_period_ptr[parabola_center[parabola_idx] +
                                                                                m_nc / 2].tuple_value;
                            intersection_point -=
                                    m_lambda_c * parabola_center[parabola_idx] * parabola_center[parabola_idx];
                            intersection_point /= 2.0 * m_lambda_c * phase - parabola_center[parabola_idx];

                            if (intersection_point <= intersection_points[parabola_idx])
                                parabola_idx--;
                            else
                                break;
                        }

                        parabola_idx++;

                        parabola_center[parabola_idx] = phase;
                        intersection_points[parabola_idx] = intersection_point;
                        intersection_points[parabola_idx + 1] = maxz;
                    }
                    parabola_idx = 0;
                    dataset_tuple_ptr = dataset_period_ptr;

                    for (phase = first_phase; phase < -first_phase; phase++, dataset_tuple_ptr++) {
                        while (intersection_points[parabola_idx + 1] < (double) phase) {
                            parabola_idx++;
                        }
                        const phase_type new_phase = parabola_center[parabola_idx];
                        const period_type period_ = periods[period_idx];
                        const double phase_dist = phase - new_phase;

                        dataset_tuple_ptr->tuple_value = (energy_type) (
                                dataset_period_ptr[new_phase + m_nc / 2].tuple_value +
                                m_lambda_c * phase_dist * phase_dist);
                        dataset_tuple_ptr->c = get_real_phase(new_phase, period_);
                        dataset_tuple_ptr->d = period_idx;
                    }
                }

                dataset_period_ptr = dataset_row_ptr;

                for (size_t phase_idx = 0; phase_idx < m_nc; phase_idx++, dataset_period_ptr++) {
                    parabola_idx = 0;
                    parabola_center[0] = 0;
                    intersection_points[0] = -maxz;
                    intersection_points[1] = maxz;

                    dataset_tuple_ptr = dataset_period_ptr + m_nc;

                    for (period_idx = 1; period_idx < m_nd; period_idx++, dataset_tuple_ptr += m_nc) {
                        const period_type d = periods[period_idx];
                        numerator = dataset_tuple_ptr->tuple_value + m_lambda_d * d * d;
                        while (true) {
                            intersection_point = (numerator -
                                                  (dataset_period_ptr[m_nc *
                                                                      parabola_center[parabola_idx]].tuple_value +
                                                   m_lambda_d * periods[parabola_center[parabola_idx]] *
                                                   periods[parabola_center[parabola_idx]])) /
                                                 (2.0 * m_lambda_d * (d - periods[parabola_center[parabola_idx]]));
                            if (intersection_point <= intersection_points[parabola_idx])
                                parabola_idx--;
                            else
                                break;
                        }
                        parabola_idx++;
                        parabola_center[parabola_idx] = period_idx;

                        intersection_points[parabola_idx] = intersection_point;
                        intersection_points[parabola_idx + 1] = maxz;
                    }

                    parabola_idx = 0;

                    dataset_tuple_ptr = dataset_period_ptr;

                    for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_tuple_ptr += m_nc) {
                        const period_type old_period = periods[period_idx];
                        while (intersection_points[parabola_idx + 1] < old_period) {
                            parabola_idx++;
                        }
                        const int new_period_idx = parabola_center[parabola_idx];
                        const double period_dist = old_period - periods[new_period_idx];

                        dataset_tuple_ptr->tuple_value = dataset_period_ptr[m_nc * new_period_idx].tuple_value +
                                                         m_lambda_d * period_dist * period_dist;
                        dataset_tuple_ptr->c = dataset_period_ptr[m_nc * new_period_idx].c;
                        dataset_tuple_ptr->d = (period_idx_type) new_period_idx;
                    }
                }
            }
        }
        dataset_row_ptr = m_dataset_ptr + (image_height - 1) * row_size;
        dataset_period_ptr = dataset_row_ptr;
        size_t row_number = image_height - 1;
        data_type *pBestNode = dataset_period_ptr;
        phase_type best_phase;
        period_type best_period;

        for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
            const period_type d = periods[period_idx];
            dataset_tuple_ptr = dataset_period_ptr;
            for (phase = -m_nc / 2; phase < m_nc / 2; phase++, dataset_tuple_ptr++)
                if (dataset_tuple_ptr->tuple_value < pBestNode->tuple_value) {
                    pBestNode = dataset_tuple_ptr;
                    best_phase = phase;
                    best_period = d;
                }
        }
        best_nodes.at(row_number) = std::make_pair(best_phase, best_period);


        while (row_number-- > 0) {
            dataset_tuple_ptr = pBestNode - row_size;

            dataset_row_ptr -= row_size;

            pBestNode = dataset_row_ptr + dataset_tuple_ptr->d * m_nc + m_nc / 2 + dataset_tuple_ptr->c;

            best_phase = dataset_tuple_ptr->c;
            best_period = periods[dataset_tuple_ptr->d];

            best_nodes.at(row_number) = std::make_pair(best_phase, best_period);
        }
        return best_nodes;
    }

    // TODO: cache values
    int const CropRowDetector::period_min(const phase_type phase, period_type *periods){
        period_type period_min;
        if (phase < 0) {
            period_min = (2 * -phase);
        } else {
            period_min = (2 * (phase + 1));
        }
        int period_idx;
        // for (period = m_mind; period < period_min; period *= m_dstep);
        for (period_idx = 0; periods[period_idx] < period_min; period_idx++);
        return period_idx;
    }

    void CropRowDetector::teardown() {
        delete[] m_dataset_ptr;
    }

}

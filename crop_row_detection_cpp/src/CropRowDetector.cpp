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
        m_dataset_ptr = new data_type[(1 + intensity_map.rows) * m_nd * m_nc];

        data_type *dataset_period_ptr = m_dataset_ptr;
        data_type *dataset_tuple_ptr;

        phase_type phase;
        for (uint period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
            dataset_tuple_ptr = dataset_period_ptr;
            for (phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                dataset_tuple_ptr->tuple_value = 0;
            }
        }
        m_period_scale_factor = .125;
        m_half_width = intensity_map.cols / 2;


        // dynamic programming setup
        m_image_height = (size_t) intensity_map.rows;
        m_search_space_length = (size_t) std::max(m_nd, m_nc);
        m_parabola_center = new phase_type[m_search_space_length];
        m_intersection_points = new energy_type[m_search_space_length + 1];
        m_best_nodes = std::vector<old_tuple_type>(m_image_height);

        m_periods = new period_type[m_nd];
        // TODO: interleave data struct

        double period = m_mind;
        for (uint period_idx = 0; period_idx < m_nd; period_idx++, period *= m_dstep) {
            m_periods[period_idx] = (period_type) period;
        }
    }

    void
    CropRowDetector::calculate_energy_integral(const std::vector<std::vector<std::vector<energy_type>>> &energy_map,
                                               const std::vector<energy_type> &max_by_row, size_t row_number,
                                               data_type *dataset_row_ptr) const {
        energy_type b_val;
        data_type *dataset_tuple_ptr;
        data_type *dataset_period_ptr;

        period_idx_type period_idx;
        phase_type phase;

        energy_type Dnrm = max_by_row[row_number];
        dataset_period_ptr = dataset_row_ptr;

        for (period_idx = 0; period_idx < this->m_nd; period_idx++, dataset_period_ptr += this->m_nc) {
            dataset_tuple_ptr = dataset_period_ptr;
            const period_type period = this->m_periods[period_idx];
            for (phase = this->m_first_phase; phase < -this->m_first_phase; phase++, dataset_tuple_ptr++) {
                const phase_type real_phase = this->get_real_phase(phase, period);
                const energy_type D = energy_map.at(row_number).at(period_idx).at(real_phase - this->m_first_phase);

                if (Dnrm >= 1.0) {
                    b_val = std::min((energy_type) (1.0 - (D / Dnrm)), this->m_maxD);
                } else {
                    b_val = this->m_maxD;
                }
                b_val += (dataset_tuple_ptr - this->m_row_size)->tuple_value;
                dataset_tuple_ptr->tuple_value = b_val;
            }
        }
    }

    std::vector<energy_type> CropRowDetector::template_matching(
            std::vector<std::vector<std::vector<energy_type>>> &energy_map,
            const cv::Mat &Intensity,
            const double positive_pulse_width,
            const double negative_pulse_width,
            const size_t window_width // w
    ) {
        uint image_height = (uint) Intensity.size[0];

        double energy = 0;
        double best_energy = -1;

        old_tuple_type best_pair;
        std::vector<energy_type> best_pairs(image_height);

        // initialize 3d vector
        energy_map.resize(image_height + 1);
        for (uint i = 0; i < image_height + 1; i++) {
            energy_map.at(i).resize(m_nd);
            for (uint j = 0; j < m_nd; ++j)
                energy_map.at(i).at(j).resize(m_nc);
        }

        for (uint image_row_num = 0; image_row_num < image_height; image_row_num++) {
            for (uint period_idx = 0; period_idx < m_nd; period_idx++) {
                const period_type period = m_periods[period_idx];

                for (phase_type phase = m_first_phase; phase < -m_first_phase; phase++) {
                    energy = CrossCorrelation(image_row_num, phase, period, positive_pulse_width, negative_pulse_width,
                                              (const int) window_width);

                    energy_map[image_row_num][period_idx][phase - m_first_phase] = energy;
                    if (energy > best_energy) {
                        best_energy = energy;
                    }
                }
            }
            best_pairs.at(image_row_num) = best_energy;
            best_energy = -1;
        }
        return best_pairs;
    }

    inline double CropRowDetector::CrossCorrelation(const uint row_number,
                                                    phase_type phase, period_type period,
                                                    const double positive_pulse_width,
                                                    const double negative_pulse_width,
                                                    const int image_width) {
        // Calcolo quantità necesarrie a CrossCorrelation
        const double scale = period * m_period_scale_factor;

        const int a = cvFloor(scale * positive_pulse_width + .5);
        int b = cvFloor(scale * negative_pulse_width + .5);

        int negative_pulse_end, negative_pulse_start, positive_pulse_end, positive_pulse_start, positive_pixels, negative_pixels;
        double positive_correlation_value, negative_correlation_value;

        double halfb = .5 * b;
        double halfa = .5 * a;
        double halfd = .5 * period;

        int kStart = (int) floor((-(image_width / 2 + phase) + halfa) / period);    // offset prima onda
        int kEnd = (int) floor(
                ((image_width / 2 - phase) + halfa) / period);   // offset ultima onda prima della fine dell'immagine

        // Calcolo centro dell'onda quadra positiva e negativa
        double distance_positive_negative_pulse_center = halfd - halfb;
        double positive_pulse_center = (double) phase + (double) kStart * (double) period + m_half_width;

        // Calcolo per Onda ad inizio immagine, non calcolabile con la classica routine

        positive_pulse_start = cvFloor(positive_pulse_center - halfa);
        positive_pulse_end = positive_pulse_start + a - 1;

        negative_pulse_start = DOUBLE2INT(positive_pulse_center + distance_positive_negative_pulse_center);
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
            if (negative_pulse_start > 0)
                negative_correlation_value = cumulative_sum(row_number, negative_pulse_end)
                                             - cumulative_sum(row_number, negative_pulse_start -
                                                                          1); //tolto  -cumulative_sum(row_number, negative_pulse_start-1);
            else if (negative_pulse_start <= 0)
                negative_correlation_value = cumulative_sum(row_number, negative_pulse_end)
                                             - cumulative_sum(row_number, negative_pulse_start);
            /*if(row_number==0) {
                cout << "2 " << cumulative_sum(row_number, negative_pulse_end) << " " << -cumulative_sum(row_number, negative_pulse_start) <<
        std::cout << "p" << ++ping_cnt;
                     " " << negative_pulse_end << " " << negative_pulse_start << endl;
            }*/
            negative_pixels = negative_pulse_end - negative_pulse_start + 1;
        } else {
            negative_correlation_value = 0;
            negative_pixels = 0;
        }

        /*if(row_number==2)
            cout <<  positive_pulse_start << " " << positive_pulse_end << " " << negative_pulse_start << " " << negative_pulse_end << endl;*/

        positive_pulse_center += period;

        for (int k = kStart + 1; k < kEnd; k++, positive_pulse_center += period) {

            positive_pulse_start = cvFloor(positive_pulse_center - halfa);
            positive_pulse_end = positive_pulse_start + a - 1;

            positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                          - cumulative_sum(row_number, positive_pulse_start - 1);

            positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

            negative_pulse_start = DOUBLE2INT(positive_pulse_center + distance_positive_negative_pulse_center);
            negative_pulse_end = negative_pulse_start + b - 1;

            negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                          - cumulative_sum(row_number, negative_pulse_start - 1);

            negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
            /*if(row_number==2)
                cout <<  k << " " << positive_pulse_start << " " << positive_pulse_end << " " << negative_pulse_start << " " << negative_pulse_end << endl;*/

        }
        positive_pulse_start = cvFloor(positive_pulse_center - halfa);

        positive_pulse_end = positive_pulse_start + a - 1;

        if (positive_pulse_end >= image_width)
            positive_pulse_end = image_width - 1;
        /*if(row_number==6)
            cout << cumulative_sum(row_number, positive_pulse_end) << " " << -cumulative_sum(row_number, positive_pulse_start-1) <<endl;*/
        positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                      - cumulative_sum(row_number, positive_pulse_start - 1);

        positive_pixels += (positive_pulse_end - positive_pulse_start + 1);


        negative_pulse_start = DOUBLE2INT(positive_pulse_center + distance_positive_negative_pulse_center);
        if (negative_pulse_start < image_width) {
            negative_pulse_end = negative_pulse_start + b - 1;

            if (negative_pulse_end >= image_width)
                negative_pulse_end = image_width - 1;

            negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                          - cumulative_sum(row_number, negative_pulse_start - 1);

            negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
        }

        double score =
                (double) (negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) /
                (double) (positive_pixels * negative_pixels);
        return score;
    }


     inline const double CropRowDetector::cumulative_sum(const int v, const int start) {
        return m_integral_image.at<double>(v, start);
    }

    std::vector<old_tuple_type>
    CropRowDetector::find_best_parameters(
            const std::vector<std::vector<std::vector<energy_type>>> &energy_map,
            const std::vector<energy_type> &max_by_row
    ) {

        energy_type numerator;
        energy_type b_val;

        data_type *dataset_row_ptr = m_dataset_ptr + m_row_size;
        data_type *dataset_tuple_ptr;
        data_type *dataset_period_ptr;

        energy_type intersection_point;
        period_idx_type period_idx;

        phase_type phase;
        energy_type Dnrm;
        uint parabola_idx;
        for (size_t row_number = 0; row_number < m_image_height - 1; row_number++, dataset_row_ptr += m_row_size) {
            Dnrm = max_by_row[row_number];
            dataset_period_ptr = dataset_row_ptr;

            for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
                dataset_tuple_ptr = dataset_period_ptr;
                const period_type period = m_periods[period_idx];
                for (phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                    const phase_type real_phase = get_real_phase(phase, period);
                    const energy_type D = energy_map.at(row_number).at(period_idx).at(real_phase - m_first_phase);

                    if (Dnrm >= 1.0) {
                        b_val = cv::min((energy_type) (1.0 - (D / Dnrm)), m_maxD);
                    } else {
                        b_val = m_maxD;
                    }
                    b_val += (dataset_tuple_ptr - m_row_size)->tuple_value;
                    dataset_tuple_ptr->tuple_value = b_val;
                }
            }
            dataset_period_ptr = dataset_row_ptr;
            // #pragma omp parallel for default(none) private(period_idx, parabola_idx, numerator, dataset_period_ptr, dataset_tuple_ptr) shared(periods, row_number)
            for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
                parabola_idx = 0;
                m_parabola_center[0] = m_first_phase;
                m_intersection_points[0] = -m_maxz;
                m_intersection_points[1] = +m_maxz;
                dataset_tuple_ptr = dataset_period_ptr + 1;

                for (phase = m_first_phase + (phase_type) 1; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                    numerator = dataset_tuple_ptr->tuple_value + m_lambda_c * phase * phase;
                    while (true) {
                        intersection_point = numerator - dataset_period_ptr[m_parabola_center[parabola_idx] + m_nc / 2].tuple_value;
                        intersection_point -= m_lambda_c * (energy_type)(m_parabola_center[parabola_idx] * m_parabola_center[parabola_idx]);
                        intersection_point /= 2.0 * m_lambda_c * phase - m_parabola_center[parabola_idx];

                        if (intersection_point <= m_intersection_points[parabola_idx])
                            parabola_idx--;
                        else
                            break;
                    }

                    parabola_idx++;

                    // TODO: parabola center in stack
                    m_parabola_center[parabola_idx] = phase;
                    m_intersection_points[parabola_idx] = intersection_point;
                    m_intersection_points[parabola_idx + 1] = m_maxz;
                }
                parabola_idx = 0;
                dataset_tuple_ptr = dataset_period_ptr;

                for (phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                    while (m_intersection_points[parabola_idx + 1] < phase) {
                        parabola_idx++;
                    }
                    const phase_type new_phase = m_parabola_center[parabola_idx];
                    const period_type period_ = m_periods[period_idx];
                    const double phase_dist = phase - new_phase;

                    dataset_tuple_ptr->tuple_value = (energy_type) (
                            dataset_period_ptr[new_phase + m_nc / 2].tuple_value +
                            m_lambda_c * phase_dist * phase_dist);
                    dataset_tuple_ptr->c = get_real_phase(new_phase, period_);
                    dataset_tuple_ptr->d = period_idx;
                }
            }

            dataset_period_ptr = dataset_row_ptr;

            for (uint phase_idx = 0; phase_idx < m_nc; phase_idx++, dataset_period_ptr++) {
                parabola_idx = 0;
                m_parabola_center[0] = 0;
                m_intersection_points[0] = -m_maxz;
                m_intersection_points[1] = m_maxz;

                dataset_tuple_ptr = dataset_period_ptr + m_nc;

                for (period_idx = 1; period_idx < m_nd; period_idx++, dataset_tuple_ptr += m_nc) {
                    const period_type d = m_periods[period_idx];
                    numerator = dataset_tuple_ptr->tuple_value + m_lambda_d * d * d;
                    while (true) {
                        intersection_point =
                                (numerator - (dataset_period_ptr[m_nc * m_parabola_center[parabola_idx]].tuple_value +
                                              m_lambda_d * m_periods[m_parabola_center[parabola_idx]] *
                                              m_periods[m_parabola_center[parabola_idx]])) /
                                (2.0 * m_lambda_d * (d - m_periods[m_parabola_center[parabola_idx]]));
                        if (intersection_point <= m_intersection_points[parabola_idx])
                            parabola_idx--;
                        else
                            break;
                    }
                    parabola_idx++;
                    m_parabola_center[parabola_idx] = period_idx;

                    m_intersection_points[parabola_idx] = intersection_point;
                    m_intersection_points[parabola_idx + 1] = m_maxz;
                }

                parabola_idx = 0;

                dataset_tuple_ptr = dataset_period_ptr;

                for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_tuple_ptr += m_nc) {
                    const period_type old_period = m_periods[period_idx];
                    while (m_intersection_points[parabola_idx + 1] < old_period) {
                        parabola_idx++;
                    }
                    const int new_period_idx = m_parabola_center[parabola_idx];
                    const double period_dist = old_period - m_periods[new_period_idx];

                    dataset_tuple_ptr->tuple_value = dataset_period_ptr[m_nc * new_period_idx].tuple_value +
                                                     m_lambda_d * period_dist * period_dist;
                    dataset_tuple_ptr->c = dataset_period_ptr[m_nc * new_period_idx].c;
                    dataset_tuple_ptr->d = (period_idx_type) new_period_idx;
                }
                // std::cin >> parabola_idx;
            }
        }
        Dnrm = max_by_row[m_image_height - 1];
        dataset_period_ptr = dataset_row_ptr;

        for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
            dataset_tuple_ptr = dataset_period_ptr;
            const period_type period = m_periods[period_idx];
            for (phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                const phase_type real_phase = get_real_phase(phase, period);
                const energy_type D = energy_map.at(m_image_height).at(period_idx).at(real_phase - m_first_phase);

                if (Dnrm >= 1.0) {
                    b_val = cv::min((energy_type) (1.0 - (D / Dnrm)), m_maxD);
                } else {
                    b_val = m_maxD;
                }
                b_val += (dataset_tuple_ptr - m_row_size)->tuple_value;
                dataset_tuple_ptr->tuple_value = b_val;
            }
        }


        dataset_row_ptr = m_dataset_ptr + (m_image_height - 1) * m_row_size;
        dataset_period_ptr = dataset_row_ptr;
        size_t row_number = m_image_height - 1;
        data_type *pBestNode = dataset_period_ptr;
        phase_type best_phase;
        period_type best_period;

        for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
            const period_type d = m_periods[period_idx];
            dataset_tuple_ptr = dataset_period_ptr;
            for (phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++)
                if (dataset_tuple_ptr->tuple_value < pBestNode->tuple_value) {
                    pBestNode = dataset_tuple_ptr;
                    best_phase = phase;
                    best_period = d;
                }
        }
        m_best_nodes.at(row_number) = std::make_pair(best_phase, best_period);

        while (row_number-- > 0) {
            dataset_tuple_ptr = pBestNode - m_row_size;

            dataset_row_ptr -= m_row_size;

            pBestNode = dataset_row_ptr + dataset_tuple_ptr->d * m_nc + m_nc / 2 + dataset_tuple_ptr->c;

            best_phase = dataset_tuple_ptr->c;
            best_period = m_periods[dataset_tuple_ptr->d];

            m_best_nodes.at(row_number) = std::make_pair(best_phase, best_period);
        }
        return m_best_nodes;
    }

    // TODO: cache values
    const period_idx_type CropRowDetector::period_min(const phase_type phase, const period_type *periods) const {
        period_type period_min;
        if (phase < 0) {
            period_min = (period_type) (2 * -phase);
        } else {
            period_min = (period_type) (2 * (phase + 1));
        }
        period_idx_type period_idx;
        for (period_idx = 0; periods[period_idx] < period_min - 1 && period_idx < m_nd; period_idx++);
        return period_idx;
    }

    void CropRowDetector::teardown() {
        delete[] m_dataset_ptr;
        delete[] m_parabola_center;
        delete[] m_intersection_points;
        delete[] m_periods;
    }

}

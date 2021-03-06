#include <iostream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <ctime>
#include "CropRowDetector.h"

int DISPLAY_FPS = 0; // 1000/60;

namespace crd_cpp {
    CropRowDetector::CropRowDetector() {}

    void CropRowDetector::pre_alloc(cv::Size image_size) {
        std::cout << "cpp::loading..." << std::endl;
        m_integral_image = cv::Mat::zeros(image_size, CV_64F);
        std::cout << "cpp::loading..." << std::endl;

        // 1 + is an hack
        int data_struct_size = (1 + image_size.height) * m_nd * m_nc;
        assert(data_struct_size > 0);
        m_dataset_ptr = new data_type[(size_t) data_struct_size];

        data_type *dataset_period_ptr = m_dataset_ptr;
        data_type *dataset_tuple_ptr;

        for (int period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
            dataset_tuple_ptr = dataset_period_ptr;
            for (phase_type phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                dataset_tuple_ptr->tuple_value = 0;
            }
        }
        m_period_scale_factor = .125;
        m_half_width = image_size.width / 2;


        // dynamic programming setup
        std::cout << "dyn prog...";
        assert(m_image_height > 0);
        m_best_energy_by_row.resize((size_t) m_image_height);
        m_search_space_length = (size_t) std::max(m_nd, m_nc);
        m_parabola_center = new phase_type[m_search_space_length];
        m_intersection_points = new energy_type[m_search_space_length + 1];
        m_best_nodes = std::vector<old_tuple_type>((size_t) m_image_height);

        m_periods = new period_type[(size_t) m_nd];
        // TODO: interleave data struct

        // will be used for xcorr
        std::cout << "xcorr...";

        // double halfa;
        // double halfb;
        // double halfd;

        // m_positive_pulse_start.resize(m_nd, std::vector<int>(m_nc, 0));
        // m_positive_pulse_end.resize(m_nd, std::vector<int>(m_nc, 0));
        // m_negative_pulse_start.resize(m_nd, std::vector<int>(m_nc, 0));
        // m_negative_pulse_end.resize(m_nd, std::vector<int>(m_nc, 0));

        // m_positive_pulse_centers.resize(m_nd, std::vector<double>(m_nc, 0));

        // m_xcorr_a.resize(m_nd);
        // m_xcorr_b.resize(m_nd);


        // // double positive_correlation_value, negative_correlation_value;

        // std::vector<double> m_distance_positive_negative_pulse_center(m_nd);

        // m_kStarts.resize(m_nd, std::vector<int>(m_nc, 0));
        // m_kEnds.resize(m_nd, std::vector<int>(m_nc, 0));

        double period = m_mind;
        for (int period_idx = 0; period_idx < m_nd; period_idx++, period *= m_dstep) {
            m_periods[period_idx] = (period_type) period;
        }
        //     double scale = period * m_period_scale_factor;
        //     double period_a = cvFloor(scale * m_positive_pulse_width + .5);
        //     m_xcorr_a.at(period_idx) = period_a;
        //     halfa = period_a / 2;

        //     double period_b = cvFloor(scale * m_negative_pulse_width + .5);
        //     m_xcorr_b.at(period_idx) = period_b;

        //     halfb = period_b / 2;
        //     halfd = period / 2;

        //     // Calcolo centro dell'onda quadra positiva e negativa
        //     double distance_positive_negative_pulse_center = halfd - halfb;
        //     m_distance_positive_negative_pulse_center.at(period) = halfd - halfb;

        //     phase_type phase;
        //     for (uint phase_idx = 0; phase_idx < m_nc; phase_idx++) {
        //         phase = phase_idx - m_first_phase;

        //         int kStart = (int) floor((-(m_image_width / 2 + phase) + halfa) / period);    // offset prima onda
        //         m_kStarts.at(period_idx).at(phase_idx) = kStart;
        //        m_kEnds.at(period_idx).at(phase_idx) = (int) floor(((m_image_width / 2 - phase) + halfa) /
        //                                                           period);   // offset ultima onda prima della fine dell'immagine
        //
        //        double positive_pulse_center = (double) phase + (double) kStart * period + m_half_width;
        //        m_positive_pulse_centers.at(period_idx).at(phase_idx) = positive_pulse_center;
        //        m_positive_pulse_start.at(period_idx).at(phase_idx) = cvFloor(positive_pulse_center - halfa);
        //        m_negative_pulse_start.at(period_idx).at(phase_idx) = DOUBLE2INT(
        //                positive_pulse_center + distance_positive_negative_pulse_center);
        //
        //        m_positive_pulse_end.at(period_idx).at(phase_idx) = (int) (
        //                m_positive_pulse_start.at(period_idx).at(phase_idx) + period_a - 1);
        //        m_negative_pulse_end.at(period_idx).at(phase_idx) = (int) (
        //                m_negative_pulse_start.at(period_idx).at(phase_idx) + period_b - 1);
        //    }
        // }

        //xcorr setup
        m_energy_map.resize((uint) m_image_height);
        m_best_energy_by_row.resize((size_t) m_image_height);
        for (uint i = 0; i < (uint) m_image_height; i++) {
            m_energy_map.at(i).resize((size_t) m_nd);
            for (uint j = 0; j < (uint) m_nd; ++j)
                m_energy_map.at(i).at(j).resize((size_t) m_nc);
        }
    }

    void CropRowDetector::template_matching(cv::Mat intensity_map) {
        double energy = 0;
        double best_energy = -1;

        for (int row = 0; row < m_image_height; row++) {
            for (int column = 1; column < m_image_width; column++) {
                m_integral_image.at<double>(row, column) =
                        (double) intensity_map.at<uchar>(row, column) + m_integral_image.at<double>(row, column - 1);
            }
        }

        for (uint image_row_num = 0; image_row_num < m_image_height; image_row_num++) {
            for (period_idx_type period_idx = 0; period_idx < m_nd; period_idx++) {
                const phase_type half_band = (const phase_type) std::floor(m_periods[period_idx] / 2);
                int phase_idx = (int) (-m_first_phase - half_band);
                for (phase_type phase = -half_band; phase < half_band; phase++, phase_idx++) {
                    energy = CrossCorrelation(image_row_num, phase, period_idx);
                    m_energy_map.at(image_row_num).at(period_idx).at((size_t) phase_idx) = energy;
                    if (energy > best_energy) {
                        best_energy = energy;
                    }
                }
            }
            m_best_energy_by_row.at(image_row_num) = best_energy;
            best_energy = -1;
        }
    }

    inline double
    CropRowDetector::CrossCorrelation(const int row_number, const phase_type phase, const period_idx_type period_idx) {

        period_type period = m_periods[period_idx];
        const double scale = period * m_period_scale_factor;

        const int a = cvFloor(scale * m_positive_pulse_width + .5);
        int b = cvFloor(scale * m_negative_pulse_width + .5);

        int negative_pulse_end, negative_pulse_start, positive_pulse_end, positive_pulse_start, positive_pixels, negative_pixels;
        double positive_correlation_value, negative_correlation_value;

        double halfb = .5 * b;
        double halfa = .5 * a;
        double halfd = .5 * period;

        int kStart = (int) floor((-(m_image_width / 2 + phase) + halfa) / period);    // offset prima onda
        int kEnd = (int) floor(
                ((m_image_width / 2 - phase) + halfa) / period);   // offset ultima onda prima della fine dell'immagine

        // Calcolo centro dell'onda quadra positiva e negativa
        double distance_positive_negative_pulse_center = halfd - halfb;
        double positive_pulse_center = (double) phase + (double) kStart * (double) period + m_half_width;

        // Calcolo per Onda ad inizio immagine, non calcolabile con la classica routine

        positive_pulse_start = std::floor(positive_pulse_center - halfa);
        positive_pulse_end = positive_pulse_start + a - 1;

        negative_pulse_start = std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
        negative_pulse_end = negative_pulse_start + b - 1;

        //cout << positive_pulse_center <<  " " << halfa << " " << positive_pulse_start << " " << positive_pulse_end << " " << negative_pulse_start << " " << negative_pulse_end << endl;

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


            positive_pulse_start = std::floor(positive_pulse_center - halfa);
            positive_pulse_end = positive_pulse_start + a - 1;

            positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                          - cumulative_sum(row_number, positive_pulse_start - 1);

            positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

            negative_pulse_start = std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
            negative_pulse_end = negative_pulse_start + b - 1;

            negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                          - cumulative_sum(row_number, negative_pulse_start - 1);

            negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
        }

        positive_pulse_start = std::floor(positive_pulse_center - halfa);

        positive_pulse_end = positive_pulse_start + a - 1;

        if (positive_pulse_end >= m_image_width) {
            positive_pulse_end = m_image_width - 1;
        }
        positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                      - cumulative_sum(row_number, positive_pulse_start - 1);

        positive_pixels += (positive_pulse_end - positive_pulse_start + 1);


        negative_pulse_start = std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
        if (negative_pulse_start < m_image_width) {
            negative_pulse_end = negative_pulse_start + b - 1;

            if (negative_pulse_end >= m_image_width) {
                negative_pulse_end = m_image_width - 1;
            }
            negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                          - cumulative_sum(row_number, negative_pulse_start - 1);
            negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
        }

        return (double) (negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) /
               (double) (positive_pixels * negative_pixels);
    }


    inline const double CropRowDetector::cumulative_sum(const int v, const int start) {
        return m_integral_image.at<double>(v, start);
    }

    std::vector<old_tuple_type> CropRowDetector::find_best_parameters(
            const energy_map_type &energy_map,
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
        int parabola_idx;
        for (size_t row_number = 0; row_number < m_image_height - 1; row_number++, dataset_row_ptr += m_row_size) {
            Dnrm = max_by_row.at(row_number);
            dataset_period_ptr = dataset_row_ptr;

            for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
                dataset_tuple_ptr = dataset_period_ptr;
                const period_type period = m_periods[period_idx];
                for (phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                    const phase_type real_phase = get_real_phase(phase, period);
                    const energy_type D = energy_map.at(row_number).at(period_idx).at((uint) (real_phase - m_first_phase));
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
                        intersection_point =
                                numerator - dataset_period_ptr[m_parabola_center[parabola_idx] + m_nc / 2].tuple_value;
                        intersection_point -= m_lambda_c * (energy_type) (m_parabola_center[parabola_idx] *
                                                                          m_parabola_center[parabola_idx]);
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

            for (int phase_idx = 0; phase_idx < m_nc; phase_idx++, dataset_period_ptr++) {
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
            }
        }
        // last row is a special case
        const size_t last_row = (const size_t) (m_image_height - 1);
        Dnrm = max_by_row.at(last_row);
        dataset_period_ptr = dataset_row_ptr;

        // calculate values for last row
        for (period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
            dataset_tuple_ptr = dataset_period_ptr;
            const period_type period = m_periods[period_idx];
            for (phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                const phase_type real_phase = get_real_phase(phase, period);
                if(real_phase - m_first_phase < 0){
                    std::cout << "WAAAAAAAAAAAAAAAAAAAAAAAAAAAA assert " << real_phase - m_first_phase << " > 0" << std::endl;
                }
                const energy_type D = energy_map.at(last_row).at(period_idx).at((uint) real_phase - m_first_phase);

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
        size_t row_number = last_row;
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

    void CropRowDetector::teardown() {
        delete[] m_dataset_ptr;
        delete[] m_parabola_center;
        delete[] m_intersection_points;
        delete[] m_periods;
    }

}

//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include "CropRowDetector.h"

// MACRO per fixing del biased rounding
#define DOUBLE2INT(x)    (int)floor(x + 0.5)

namespace crd_cpp {
    struct GT_Residual {
        GT_Residual(double x, double y, double x0, double frequency_factor)
                : x_(x), y_(y), x0_(x0), frequency_factor_(frequency_factor) {}

        template<typename T>
        bool operator()(const T *p, const T *pf, const T *f, T *residual) const {

            //funzione residuo da minimizzare
            residual[0] = T(y_) - ((pf[1] + T(x0_) * pf[0]) * p[0] * pow(T(x_), 4) +
                                   (pf[3] + T(x0_) * pf[2]) * p[1] * pow(T(x_), 3) +
                                   (pf[5] + T(x0_) * pf[4]) * p[2] * pow(T(x_), 2) +
                                   (pf[7] + T(x0_) * pf[6]) * p[3] * pow(T(x_), 1) +
                                   p[4] + T(frequency_factor_) * f[0]);
            return true;
        }
        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(double x,
                                           double y,
                                           double x0,
                                           double frequency_factor) {
            return (new ceres::AutoDiffCostFunction<GT_Residual, 1,5,8,1>(
                    new GT_Residual(x, y, x0, frequency_factor)));
        }
        double x_;
        double y_;
        double x0_;
        double frequency_factor_;
    };
    class CropRowCostFunctor {
    public:
        CropRowCostFunctor(const cv::Mat& image, int poly_idx) : image_(image), poly_idx_(poly_idx){}

        bool operator()(const double poly_coeffs[8], const double prespective_coeff[5],
                       const double* poly_period, double *residuals) const {
            uint sum = 0;
            for (uint row_num = 0; row_num < image_.rows; row_num++) {
                for (int poly_idx = -1; poly_idx < 2; poly_idx++) {
                    sum += Polyfit::eval_poly(row_num, poly_idx, poly_coeffs, prespective_coeff, poly_period);
                }
            }
            residuals[0] = sum;
            return true;
        }

        static ceres::CostFunction* Create(const cv::Mat image, int poly_idx) {
            return new ceres::NumericDiffCostFunction<CropRowCostFunctor, ceres::CENTRAL, 1, 5, 8, 1>(new CropRowCostFunctor(image, poly_idx));
        }
    private:
        int poly_idx_;
        cv::Mat image_;
    };
    CropRowDetector::CropRowDetector() {}

    void CropRowDetector::load(cv::Size image_size) {
        std::cout << "cpp::loading..." << std::endl;
        // cv::Mat intensity_map_64f;
        // intensity_map.convertTo(intensity_map_64f, CV_64F);
        m_integral_image = cv::Mat::zeros(image_size, CV_64F);
        std::cout << "cpp::loading..." << std::endl;

        // 1 + is an hack
        m_dataset_ptr = new data_type[(1 + image_size.height) * m_nd * m_nc];

        data_type *dataset_period_ptr = m_dataset_ptr;
        data_type *dataset_tuple_ptr;

        for (uint period_idx = 0; period_idx < m_nd; period_idx++, dataset_period_ptr += m_nc) {
            dataset_tuple_ptr = dataset_period_ptr;
            for (phase_type phase = m_first_phase; phase < -m_first_phase; phase++, dataset_tuple_ptr++) {
                dataset_tuple_ptr->tuple_value = 0;
            }
        }
        m_period_scale_factor = .125;
        m_half_width = image_size.width / 2;


        // dynamic programming setup
        std::cout << "dyn prog...";
        m_best_energy_by_row.resize(m_image_height);
        m_search_space_length = (size_t) std::max(m_nd, m_nc);
        m_parabola_center = new phase_type[m_search_space_length];
        m_intersection_points = new energy_type[m_search_space_length + 1];
        m_best_nodes = std::vector<old_tuple_type>(m_image_height);

        m_periods = new period_type[m_nd];
        // TODO: interleave data struct

        // will be used for xcorr
        std::cout << "xcorr...";

        double halfa;
        double halfb;
        double halfd;

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
        for (uint period_idx = 0; period_idx < m_nd; period_idx++, period *= m_dstep) {
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
        m_energy_map.resize(m_image_height);
        for (uint i = 0; i < m_image_height; i++) {
            m_energy_map.at(i).resize(m_nd);
            for (uint j = 0; j < m_nd; ++j)
                m_energy_map.at(i).at(j).resize(m_nc);
        }
        m_best_energy_by_row.resize(m_image_height);
        // Calcolo quantità necesarrie a CrossCorrelation
    }

    /*
    void CropRowDetector::calculate_energy_integral(const std::vector<std::vector<std::vector<energy_type>>> &energy_map,
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
     */

    void CropRowDetector::template_matching(cv::Mat intensity_map) {
        double energy = 0;
        double best_energy = -1;

        for (int row = 0; row < m_image_height; row++) {
            // m_integral_image.at<double>(row, 0) = (double) intensity_map.at<uchar>(row, 0);
            for (int column = 1; column < m_image_width; column++) {
                m_integral_image.at<double>(row, column) =
                        (double) intensity_map.at<uchar>(row, column) + m_integral_image.at<double>(row, column - 1);
            }
        }

        for (uint image_row_num = 0; image_row_num < m_image_height; image_row_num++) {
            for (period_idx_type period_idx = 0; period_idx < m_nd; period_idx++) {
                const phase_type half_band = (const phase_type) std::floor(m_periods[period_idx] / 2);
                int phase_idx = - m_first_phase - half_band;
                for (phase_type phase = -half_band; phase < half_band; phase++, phase_idx++) {
                    energy = CrossCorrelation(image_row_num, phase, period_idx);
                    m_energy_map.at(image_row_num).at(period_idx).at(phase_idx) = energy;
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
    CropRowDetector::CrossCorrelation(const uint row_number, const phase_type phase, const period_idx_type period_idx) {

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
        int kEnd = (int) floor(((m_image_width / 2 - phase) + halfa) / period);   // offset ultima onda prima della fine dell'immagine

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
					-cumulative_sum(row_number, negative_pulse_start); //tolto  -cumulative_sum(row_number, negative_pulse_start-1);

            negative_pixels = negative_pulse_end - negative_pulse_start + 1;
        }
    else {
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
    if(negative_pulse_start < m_image_width)
    {
            negative_pulse_end = negative_pulse_start + b - 1;

            if (negative_pulse_end >= m_image_width) {
                negative_pulse_end = m_image_width - 1;
            }
            negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                          - cumulative_sum(row_number, negative_pulse_start - 1);
            negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
        }

    return (double)(negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) / (double)(positive_pixels * negative_pixels);
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
        uint parabola_idx;
        for (size_t row_number = 0; row_number < m_image_height - 1; row_number++, dataset_row_ptr += m_row_size) {
            Dnrm = max_by_row.at(row_number);
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
                const energy_type D = energy_map.at(last_row).at(period_idx).at(real_phase - m_first_phase);

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

    Polyfit::Polyfit(cv::Mat image, cv::Mat intensity_map, std::vector<crd_cpp::old_tuple_type> ground_truth) {
        int crop_row_center;
        double crop_row_period;

        m_options.max_num_iterations = 50;
        m_options.linear_solver_type = ceres::DENSE_QR;
        m_options.minimizer_progress_to_stdout = false;

        int image_center = image.cols / 2;
        int image_height = image.rows;

        int poly_origin_center = image_center + ground_truth.at(0).first;
        double poly_origin_period = ground_truth.at(0).second;

        int poly_idx;
        ceres::CostFunction *cost_function;

        for (uint row_num = 0; row_num < image_height; row_num++) {
            crop_row_center = image_center + ground_truth.at(row_num).first;
            crop_row_period = ground_truth.at(row_num).second;
            for(poly_idx = -1; poly_idx < 2; poly_idx++){
                // cost_function = new ceres::NumericDiffCostFunction<CropRowCostFunctor, ceres::CENTRAL, 1, 5, 8, 1>(new CropRowCostFunctor(image, poly_idx));
                // m_problem.AddResidualBlock(cost_function, NULL, m_polynomial, m_perspective_factors, &m_poly_period);
                cost_function = GT_Residual::Create(row_num,
                                                    crop_row_center + poly_idx * crop_row_period,
                                                    poly_origin_center + poly_idx * poly_origin_period,
                                                    poly_idx);
                m_problem.AddResidualBlock(cost_function, NULL, m_polynomial, m_perspective_factors, &m_poly_period);
            }
        }

        ceres::Solve(m_options, &m_problem, &m_summary);


        std::cout << m_polynomial[0] << " " << m_polynomial[1] << " " << m_polynomial[2] << " " << m_polynomial[3] << " " <<
                  m_polynomial[4] << " " << m_poly_period << '\n';

        std::cout << m_perspective_factors[0] << " " << m_perspective_factors[1] << " " << m_perspective_factors[2] << " "
                  << m_perspective_factors[3] << " " <<
                  m_perspective_factors[4] << " " << m_perspective_factors[5] << " " << m_perspective_factors[6] << " "
                  << m_perspective_factors[7] << '\n';

        plot_polys(image, m_polynomial, m_perspective_factors, m_poly_period, image, poly_origin_center, poly_origin_period);

        cv::Size ksize = cv::Size(31, 3);
        double sigmaX = 20;
        std::cout << "blurring map" << std::endl;
        cv::imshow("pre blur", intensity_map);
        cv::waitKey(0);
        cv::GaussianBlur(intensity_map, intensity_map, ksize, sigmaX);
        // cv::blur(intensity_map, intensity_map, ksize);

        m_image = image;
        cv::imshow("post blur", intensity_map * 2);
        cv::waitKey(0);
        m_intensity_map = intensity_map;
    }
    void Polyfit::plot_polys(const cv::Mat &inpImg_, const double *polynomial, const double *perspective_factors,
                             double poly_period, const cv::Mat &drawImg_, int poly_origin_center, double poly_origin_period) {
        int poly_idx = 0;
        double x0 = poly_origin_center + poly_idx * poly_origin_period;

        poly_idx = 1;
        double x0_right = poly_origin_center + poly_idx * poly_origin_period;

        poly_idx = 2;
        double x0_2right = poly_origin_center + poly_idx * poly_origin_period;

        poly_idx = 3;
        double x0_3right = poly_origin_center + poly_idx * poly_origin_period;

        poly_idx = -1;
        double x0_left = poly_origin_center + poly_idx * poly_origin_period;

        poly_idx = -2;
        double x0_2left = poly_origin_center + poly_idx * poly_origin_period;

        for (uint image_row_num = 0; image_row_num < inpImg_.rows; image_row_num++) {
            draw_crop(polynomial, perspective_factors, drawImg_, x0, image_row_num);
            circle(drawImg_, cv::Point2f(
                    ((perspective_factors[1] + x0_right * perspective_factors[0]) * polynomial[0] *
                     pow(image_row_num, 4) +
                     (perspective_factors[3] + x0_right * perspective_factors[2]) * polynomial[1] *
                     pow(image_row_num, 3) +
                     (perspective_factors[5] + x0_right * perspective_factors[4]) * polynomial[2] *
                     pow(image_row_num, 2) +
                     (perspective_factors[7] + x0_right * perspective_factors[6]) * polynomial[3] * image_row_num +
                     polynomial[4] + poly_period), image_row_num), 2, cv::Scalar(0, 255, 0), 2);
            circle(drawImg_, cv::Point2f(
                    ((perspective_factors[1] + x0_left * perspective_factors[0]) * polynomial[0] *
                     pow(image_row_num, 4) +
                     (perspective_factors[3] + x0_left * perspective_factors[2]) * polynomial[1] *
                     pow(image_row_num, 3) +
                     (perspective_factors[5] + x0_left * perspective_factors[4]) * polynomial[2] *
                     pow(image_row_num, 2) +
                     (perspective_factors[7] + x0_left * perspective_factors[6]) * polynomial[3] * image_row_num +
                     polynomial[4] - poly_period), image_row_num), 2, cv::Scalar(0, 255, 0), 2);
            circle(drawImg_, cv::Point2f(
                    ((perspective_factors[1] + x0_2right * perspective_factors[0]) * polynomial[0] *
                     pow(image_row_num, 4) +
                     (perspective_factors[3] + x0_2right * perspective_factors[2]) * polynomial[1] *
                     pow(image_row_num, 3) +
                     (perspective_factors[5] + x0_2right * perspective_factors[4]) * polynomial[2] *
                     pow(image_row_num, 2) +
                     (perspective_factors[7] + x0_2right * perspective_factors[6]) * polynomial[3] * image_row_num +
                     polynomial[4] + 2 * poly_period), image_row_num), 2, cv::Scalar(255, 0, 0), 2);
            circle(drawImg_, cv::Point2f(
                    ((perspective_factors[1] + x0_2left * perspective_factors[0]) * polynomial[0] *
                     pow(image_row_num, 4) +
                     (perspective_factors[3] + x0_2left * perspective_factors[2]) * polynomial[1] *
                     pow(image_row_num, 3) +
                     (perspective_factors[5] + x0_2left * perspective_factors[4]) * polynomial[2] *
                     pow(image_row_num, 2) +
                     (perspective_factors[7] + x0_2left * perspective_factors[6]) * polynomial[3] * image_row_num +
                     polynomial[4] - 2 * poly_period), image_row_num), 2, cv::Scalar(255, 0, 0), 2);
            circle(drawImg_, cv::Point2f(
                    ((perspective_factors[1] + x0_3right * perspective_factors[0]) * polynomial[0] *
                     pow(image_row_num, 4) +
                     (perspective_factors[3] + x0_3right * perspective_factors[2]) * polynomial[1] *
                     pow(image_row_num, 3) +
                     (perspective_factors[5] + x0_3right * perspective_factors[4]) * polynomial[2] *
                     pow(image_row_num, 2) +
                     (perspective_factors[7] + x0_3right * perspective_factors[6]) * polynomial[3] * image_row_num +
                     polynomial[4] + 3 * poly_period), image_row_num), 2, cv::Scalar(255, 0, 0), 2);
        }
        imshow("drawImg_", drawImg_);
        //     cv::imwrite("/home/temistocle/Scrivania/ciao.jpg", drawImg_);
        cv::waitKey(0);
    }
    void Polyfit::plot_fitted_polys() {
        cv::Mat img = m_image.clone();
        cv::Scalar_<double> color;

        for (uint image_row_num = 0; image_row_num < m_image.rows; image_row_num++) {
            for(int poly_idx = -2; poly_idx < 4; poly_idx++){
                int column = eval_poly(image_row_num, poly_idx);

                if(std::abs(poly_idx) < 2)
                    color = cv::Scalar(0, 0, 255);
                else
                    color = cv::Scalar(127, 127, 127);

                cv::circle(img, cv::Point2f(column, image_row_num), 2, color, 1);
            }
        }
        cv::imshow("poly_fitted", img);
        cv::waitKey(0);
    }

    const int Polyfit::eval_poly(uint image_row_num, int poly_idx, const double m_polynomial[5],
                                 const double m_perspective_factors[8], const double* m_poly_period)
    {
        int column;
        double poly_origin = m_polynomial[4] + *(m_poly_period) * poly_idx;
        column = (int) ((m_perspective_factors[1] + poly_origin * m_perspective_factors[0]) * m_polynomial[0] * pow(image_row_num, 4)
                        + (m_perspective_factors[3] + poly_origin * m_perspective_factors[2]) * m_polynomial[1] * pow(image_row_num, 3)
                        + (m_perspective_factors[5] + poly_origin * m_perspective_factors[4]) * m_polynomial[2] * pow(image_row_num, 2)
                        + (m_perspective_factors[7] + poly_origin * m_perspective_factors[6]) * m_polynomial[3] * image_row_num
                        + m_polynomial[4]
                        + poly_idx * *(m_poly_period));

        return column;
    }
    int Polyfit::eval_poly(uint image_row_num, int poly_idx){
        int column;
        double poly_origin = m_polynomial[4] + m_poly_period * poly_idx;
        column = (int) ((m_perspective_factors[1] + poly_origin * m_perspective_factors[0]) * m_polynomial[0] * pow(image_row_num, 4)
                        + (m_perspective_factors[3] + poly_origin * m_perspective_factors[2]) * m_polynomial[1] * pow(image_row_num, 3)
                        + (m_perspective_factors[5] + poly_origin * m_perspective_factors[4]) * m_polynomial[2] * pow(image_row_num, 2)
                        + (m_perspective_factors[7] + poly_origin * m_perspective_factors[6]) * m_polynomial[3] * image_row_num
                        + m_polynomial[4]
                        + poly_idx * m_poly_period);

        return column;
    }
    void Polyfit::draw_crop(const double *polynomial, const double *perspective_factors, const cv::Mat &drawImg_, double x0, uint image_row_num) {
        auto x = (perspective_factors[1] + x0 * perspective_factors[0]) * polynomial[0] * pow(image_row_num, 4) +
                 (perspective_factors[3] + x0 * perspective_factors[2]) * polynomial[1] * pow(image_row_num, 3) +
                 (perspective_factors[5] + x0 * perspective_factors[4]) * polynomial[2] * pow(image_row_num, 2) +
                 (perspective_factors[7] + x0 * perspective_factors[6]) * polynomial[3] * image_row_num + polynomial[4];

        auto point = cv::Point2f((float) x, image_row_num);
        circle(drawImg_, point, 2, cv::Scalar(0, 255, 0), 2);
    }
    void Polyfit::add_noise() {
        std::default_random_engine generator;

        double fract = 10 * 2;
        double mean;
        double sigma;
        plot_fitted_polys();
        for (uint i = 1; i < 5; i++) {
            mean = m_polynomial[i];
            sigma = mean / fract;
            std::normal_distribution<double> distribution(mean, sigma);
            auto val = distribution(generator);
            std::cout << mean << ": " << val << std::endl;
            m_polynomial[i] = val;
        }
        {
            mean = m_polynomial[0];
            sigma = mean / (fract * 10);
            std::normal_distribution<double> distribution(mean, sigma);
            auto val = distribution(generator);
            std::cout << mean << ": " << val << std::endl;
            m_polynomial[0] = val;
        }

        for(uint i = 0; i < 8; i++){
            mean = m_perspective_factors[i];
            sigma = mean / fract;
            std::normal_distribution<double> distribution(mean, sigma);
            auto val = distribution(generator);
            std::cout << mean << ": " << val << std::endl;
            m_perspective_factors[i] = val;
        }
        {
            mean = m_poly_period;
            sigma = mean / fract * 10;
            std::normal_distribution<double> distribution(mean, sigma);
            auto val = distribution(generator);
            m_poly_period = val;
        }
        plot_fitted_polys();
    }
    void Polyfit::fit(){
    }
}

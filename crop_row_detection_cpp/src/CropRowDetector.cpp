#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include "CropRowDetector.h"
#include <boost/scoped_ptr.hpp>

int DISPLAY_FPS = 0; // 1000/60;

namespace crd_cpp {
    class PointResidual {
    public:
        static ceres::CostFunction *Create(double x, double y, int poly_idx) {
            return (new ceres::AutoDiffCostFunction<PointResidual, 1, 5, 8, 1>(new PointResidual(x, y, poly_idx)));
        }

        template<typename T>
        bool operator()(const T *poly_coeff, const T *perspective_coeff, const T *poly_period, T *residual) const {
            residual[0] = T(target_column)
                          - eval_poly_double(row_num_, poly_idx_, poly_coeff, perspective_coeff, poly_period);
            // residual[0] = std::min(residual[0], 10);
            return true;
        }
    public:
        template<typename T>
        const T eval_poly_double(double image_row_num, double poly_idx, const T *polynomial,
                                               const T *perspective_factors, const T *poly_period) const {
            T column;
            T poly_origin = polynomial[4] + *(poly_period) * poly_idx;
            column = (
                    // (perspective_factors[1] + poly_origin * perspective_factors[0]) * polynomial[0] * pow(T(image_row_num), 4)
                    + (perspective_factors[3] + poly_origin * perspective_factors[2]) * polynomial[1] * pow(T(image_row_num), 3)
                    + (perspective_factors[5] + poly_origin * perspective_factors[4]) * polynomial[2] * pow(T(image_row_num), 2)
                    + (perspective_factors[7] + poly_origin * perspective_factors[6]) * polynomial[3] * T(image_row_num)
                    + polynomial[4]
                    + T(poly_idx) * *(poly_period)
            );
            return column;
        }
        PointResidual(double row_num, double col_num, int poly_idx)
                : row_num_(row_num), target_column(col_num), poly_idx_(poly_idx) {}
        double row_num_;
        double target_column;
        double poly_idx_;
    };

    class CropRowCostFunctor {
    public:
        bool operator()(const double *poly_coeffs, const double *prespective_coeff,
                        const double *poly_period, double *residuals) const {
            const double x = Polyfit::eval_poly_double(row_number_, poly_idx_, poly_coeffs, prespective_coeff, poly_period);
            const int left = (const int) std::floor(x);
            const double f_left = ((int) image_.at<uchar>(row_number_, left));
            // left + 1 almost always exists
            const double f_right = ((int) image_.at<uchar>(row_number_, left + 1));
            const double loss = f_left + (f_right - f_left) * (x - left);
            residuals[0] = -loss;
            return true;
        }

        static ceres::CostFunction *Create(const cv::Mat image, int poly_idx, int row_number) {
            return new ceres::NumericDiffCostFunction<CropRowCostFunctor, ceres::CENTRAL, 1, 5, 8, 1>(
                    new CropRowCostFunctor(image, poly_idx, row_number));
        }

    private:
        CropRowCostFunctor(const cv::Mat &image, int poly_idx, int row_number) :
                image_(image), poly_idx_(poly_idx), row_number_(row_number) {}

        cv::Mat image_;
        int poly_idx_;
        int row_number_;
    };

    CropRowDetector::CropRowDetector() {}

    void CropRowDetector::pre_alloc(cv::Size image_size) {
        std::cout << "cpp::loading..." << std::endl;
        // cv::Mat intensity_map_64f;
        // intensity_map.convertTo(intensity_map_64f, CV_64F);
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
            // m_integral_image.at<double>(row, 0) = (double) intensity_map.at<uchar>(row, 0);
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

    Polyfit::Polyfit(cv::Mat image,
                     cv::Mat intensity_map,
                     std::vector<crd_cpp::old_tuple_type> ground_truth,
                     cv::Mat &out_img) {
        m_image = image.clone();
        m_image_center = image.cols / 2;
        m_image_height = image.rows;
        m_intensity_map = intensity_map;
        m_crop_row_points = std::vector<cv::Point2f>((size_t) image.rows * 3);
        m_spammable_img = out_img;

        std::reverse(ground_truth.begin(), ground_truth.end());
        fit_poly_on_points(ground_truth);
        calculate_poly_points(); // not really needded

        cv::Size ksize = cv::Size(0, 0);
        double sigmaX = 1;
        std::cout << "[plotting] pre blur" << std::endl;
        cv::imshow("post-blurr", intensity_map);
        cv::waitKey(0);
        cv::destroyAllWindows();

        cv::GaussianBlur(intensity_map, intensity_map, ksize, sigmaX);
        cv::GaussianBlur(m_spammable_img, intensity_map, ksize, sigmaX);

        std::cout << "[plotting] post blur" << std::endl;
        cv::imshow("post-blurr", m_spammable_img);
        cv::waitKey(0);
        cv::destroyAllWindows();

        plot_fitted_polys("initial fit");
        fit_poly_on_image(intensity_map);
        plot_fitted_polys("imap fit");
    }
    void Polyfit::fit_poly_on_image(cv::Mat new_intensity_map) {
        ceres::Solver::Options m_options;
        ceres::Problem m_problem;
        ceres::Solver::Summary m_summary;

        m_options.max_num_iterations = 100;
        m_options.function_tolerance = 1e-14;
        m_options.parameter_tolerance = 1e-14;
        m_options.linear_solver_type = ceres::DENSE_QR;
        m_options.minimizer_progress_to_stdout = true;

        for (int row_num = 0; row_num < m_image_height; row_num++) {
            for (int poly_idx = -1; poly_idx < 2; poly_idx++) {
                ceres::CostFunction *cost_function = CropRowCostFunctor::Create(new_intensity_map, poly_idx, row_num);
                m_problem.AddResidualBlock(cost_function, NULL, m_polynomial, m_perspective_factors, &m_poly_period);
            }
        }
        ceres::Solve(m_options, &m_problem, &m_summary);
        std::cout << m_summary.FullReport() << std::endl;
        std::cout << "solved" << std::endl;
    }

    void Polyfit::fit_poly_on_points(std::vector<crd_cpp::old_tuple_type> points) {
        ceres::Solver::Options m_options;
        ceres::Problem m_problem;
        ceres::Solver::Summary m_summary;

        m_options.max_num_iterations = 50;
        m_options.function_tolerance = 1e-10;
        m_options.parameter_tolerance = 1e-14;
        m_options.linear_solver_type = ceres::DENSE_QR;
        m_options.minimizer_progress_to_stdout = false;

        period_type crop_row_center, crop_row_period;
        int poly_idx;
        ceres::CostFunction *cost_function;

        for (int row_num = 0; row_num < m_image_height; row_num++) {
            crop_row_center = m_image_center + points.at((size_t) row_num).first;
            crop_row_period = points.at((size_t) row_num).second;
            for (poly_idx = -1; poly_idx < 2; poly_idx++) {
                cost_function = PointResidual::Create(row_num, crop_row_center + poly_idx * crop_row_period, poly_idx);
                m_problem.AddResidualBlock(cost_function, NULL, m_polynomial, m_perspective_factors, &m_poly_period);
            }
        }
        ceres::Solve(m_options, &m_problem, &m_summary);
    }


    void Polyfit::fit_poly_on_points() {
        ceres::Solver::Options m_options;
        ceres::Problem m_problem;
        ceres::Solver::Summary m_summary;

        m_options.max_num_iterations = 50;
        m_options.function_tolerance = 1e-10;
        m_options.parameter_tolerance = 1e-14;
        m_options.linear_solver_type = ceres::DENSE_QR;
        m_options.minimizer_progress_to_stdout = false;

        int poly_idx;
        ceres::CostFunction *cost_function;
        const int limit = m_image_height/2;
        std::cout << "LIMIT: " << limit << std::endl;
        for (int row_num = 0; row_num < limit; row_num++) {
            for (poly_idx = -1; poly_idx < 2; poly_idx++) {
                auto p = m_crop_row_points.at((size_t) (row_num * 3 + (1 + poly_idx)));
                cost_function = PointResidual::Create(p.y, p.x, poly_idx);
                m_problem.AddResidualBlock(cost_function, NULL, m_polynomial, m_perspective_factors, &m_poly_period);
            }
        }
        ceres::Solve(m_options, &m_problem, &m_summary);
    }

    void Polyfit::plot_fitted_polys(std::string suffix) {
        std::cout << "[plotting]: " << suffix << std::endl;
        cv::Mat img = m_image.clone();
        cv::Scalar_<double> color;
        for (int image_row_num = 0; image_row_num < m_image_height; image_row_num++) {
            for (int poly_idx = -2; poly_idx < 4; poly_idx++) {
                int column = eval_poly(image_row_num, poly_idx);

                if (std::abs(poly_idx) < 2)
                    color = cv::Scalar(0, 0, 255);
                else
                    color = cv::Scalar(127, 127, 127);

                cv::circle(img, cv::Point2f(column, m_image_height - image_row_num - 1), 2, color, 1);
                cv::circle(m_spammable_img, cv::Point2f(column, m_image_height - image_row_num - 1), 2, color, 1);
            }
        }
        cv::imshow("RGB plot_" + suffix, m_spammable_img);
        // cv::imshow("plot_" + suffix, img);
        cv::imwrite("plot_" + suffix + ".jpg", img);
        cv::waitKey(DISPLAY_FPS);
    }

    void Polyfit::plot_crop_points(std::string suffix) {
        std::cout << "[plotting]: " << suffix << std::endl;
        cv::Mat img = m_image.clone();
        for (auto p: m_crop_row_points) {
            cv::circle(img, p, 1, cv::Scalar(255, 0, 0), 1);
            cv::circle(m_spammable_img, p, 1, cv::Scalar(255, 0, 0), 1);
        }
        cv::imshow("plot_" + suffix, img);
        cv::imshow("plot_" + suffix, m_spammable_img);
        cv::imwrite("plot_" + suffix + ".jpg", img);
        cv::waitKey(DISPLAY_FPS);
        // cv::destroyWindow("plot_" + suffix);
    }

    const int Polyfit::eval_poly(int image_row_num, int poly_idx, const double polynomial[5],
                                 const double perspective_factors[8], const double *poly_period) {
        return (int) eval_poly_double(image_row_num, poly_idx, polynomial, perspective_factors, poly_period);
    }
    const double Polyfit::eval_poly_double(int image_row_num, int poly_idx, const double *polynomial,
                                           const double *perspective_factors, const double *poly_period) {
        double column;
        double poly_origin = polynomial[4] + *(poly_period) * poly_idx;
        column = (
                  // (perspective_factors[1] + poly_origin * perspective_factors[0]) * polynomial[0] * pow(image_row_num, 4)
                + (perspective_factors[3] + poly_origin * perspective_factors[2]) * polynomial[1] * pow(image_row_num, 3)
                + (perspective_factors[5] + poly_origin * perspective_factors[4]) * polynomial[2] * pow(image_row_num, 2)
                + (perspective_factors[7] + poly_origin * perspective_factors[6]) * polynomial[3] * image_row_num
                + polynomial[4]
                + poly_idx * *(poly_period)
        );
        return column;
    }

    int Polyfit::eval_poly(int image_row_num, int poly_idx) {
        return eval_poly(image_row_num, poly_idx, m_polynomial, m_perspective_factors, &m_poly_period);
    }

    void Polyfit::fit(cv::Mat new_frame) {
        cv::Mat optical_flow = calculate_flow(new_frame);
        for (cv::Point2f &p: m_crop_row_points) {
            p += optical_flow.at<cv::Point2f>(p);
        }
        // plot_crop_points("pts after opt flow");

        fit_poly_on_points();
        calculate_poly_points();

        // plot_fitted_polys("fit post opt flow effect");

        m_intensity_map = new_frame;
        fit_poly_on_image(new_frame);
        plot_fitted_polys("fit on intensity map");

    }

    void Polyfit::calculate_poly_points() {
        for (int row_num = 0; row_num < m_image_height; row_num++) {
            for (int poly_idx = -1; poly_idx < 2; poly_idx++) {
                const int column = eval_poly(row_num, poly_idx);
                assert(row_num * 3 + (1 + poly_idx) >= 0);
                m_crop_row_points.at((size_t) (row_num * 3 + (1 + poly_idx))) = cv::Point2f(column, row_num);
            }
        }
    }

    cv::Mat Polyfit::calculate_flow(const cv::Mat &new_frame) {
        // double mask_thresh = 10.0/255.0;
        // cv::Mat min_eigen, mask, optical_flow, flow_mask;
        cv::Mat optical_flow;

        // // Optional
        // //
        // cv::cornerMinEigenVal(m_intensity_map, min_eigen, 7, 3, cv::BORDER_DEFAULT);
        // // WARNING Optimize here!
        // cv::normalize(min_eigen, min_eigen, 1, 0, cv::NORM_MINMAX);
        // cv::threshold(min_eigen, min_eigen, mask_thresh, 255, cv::THRESH_BINARY);
        // // min_eigen.convertTo(mask, CV_64F);
        // min_eigen.convertTo(mask, cv::DataType<uchar>::type);

        calcOpticalFlowFarneback(m_intensity_map, new_frame, optical_flow, 0.5, 3, 15, 5, 5, 1.2,
                                 0); // TODO: optimize params
        // cv::Mat debug_flow = drawDenseOptFlow(optical_flow, m_intensity_map, 8, cv::Scalar(0, 0, 255), flow_mask);
        // cv::imshow("optical_flow" , debug_flow);
        // cv::waitKey(100);
        return optical_flow;
    }

    /**
        * @brief Provide an image with depicted sample motion vectors of the
        input dense optical flow
        *
        * @param[in] flow The dense optical flow matrix, with one motion vector
        for each pixel (2 channels, type float)
        * @param[in] img Image to be used as background for the resulting
        optical flow image (e.g., the reference image
        *            from which the flow has been computed). It must have the
        same size as flow, with one or three
        *            channels.
        * @param[in] step Sample step, in pixel (i.e, drawDenseOptFlow() will
        draw a motion vector each step pixel)
        * @param[in] color Motion vectors color
        * @param[in] mask If not empty, it must have the same size as flow, with
        one channel and type char.
        *             The motion vectors are drawn only for non-zero pixels in
        the mask.
    */
    cv::Mat Polyfit::drawDenseOptFlow(const cv::Mat &flow, const cv::Mat &img, int step,
                                      cv::Scalar color, const cv::Mat &mask) {
        if (flow.depth() != cv::DataType<float>::type || !flow.rows || !flow.cols
            || flow.channels() != 2 ||
            img.rows != flow.rows || img.cols != flow.cols || (img.channels()
                                                               != 1 && img.channels() != 3) ||
            (!mask.empty() && (mask.depth() != cv::DataType<uchar>::type ||
                               mask.channels() != 1 ||
                               mask.rows != flow.rows || mask.cols != flow.cols)))
            throw std::invalid_argument("Unsopported  or incompatible images");

        cv::Mat flow_img(flow.rows, flow.cols, cv::DataType<cv::Vec<uchar, 3>>::type);
        cv::Mat tmp_img = img;

        if (img.channels() == 1) {
            if (img.type() != cv::DataType<uchar>::type)
                img.convertTo(tmp_img, cv::DataType<uchar>::type);
            cvtColor(tmp_img, flow_img, cv::COLOR_GRAY2BGR);
        } else {
            if (img.type() != cv::DataType<cv::Vec3b>::type)
                img.convertTo(tmp_img, cv::DataType<cv::Vec3b>::type);
            tmp_img.copyTo(flow_img);
        }
        if (!mask.empty()) {
            for (int y = 0; y < flow_img.rows; y += step) {
                const uchar *mask_p = mask.ptr<uchar>(y);
                const cv::Point2f *flow_p = flow.ptr<cv::Point2f>(y);

                for (int x = 0; x < flow_img.cols; x += step, mask_p += step,
                                                   flow_p += step) {
                    if (*mask_p) {
                        const cv::Point2f &fxy = *flow_p;
                        line(flow_img, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
                    }
                }
            }
        } else {
            for (int y = 0; y < flow_img.rows; y += step) {
                const cv::Point2f *flow_p = flow.ptr<cv::Point2f>(y);
                for (int x = 0; x < flow_img.cols; x += step, flow_p += step) {
                    const cv::Point2f &fxy = *flow_p;
                    line(flow_img, cv::Point(x, y),
                         cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), color);
                }
            }
        }
        return flow_img;
    }
}

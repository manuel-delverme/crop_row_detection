//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

#include <ceres/ceres.h>

namespace crd_cpp {
    typedef int phase_type;
    typedef float period_type;
    typedef unsigned short period_idx_type;
    typedef double energy_type;
    typedef std::pair<phase_type, period_type> old_tuple_type;
    typedef std::pair<phase_type, size_t> tuple_type;
    typedef std::vector<std::vector<std::vector<energy_type> > > energy_map_type;

    struct data_type {
        energy_type tuple_value;
        phase_type c;
        period_idx_type d;
    };

    class CropRowDetector {
    public:

        CropRowDetector();

        void pre_alloc(cv::Size image_size);

        void teardown();

        inline double CrossCorrelation(
                const int row_number,
                const phase_type phase,
                const period_idx_type period_idx
        );

        std::vector<old_tuple_type> find_best_parameters(
                const energy_map_type &energy_map,
                const std::vector<energy_type> &best_pairs
        );

        void template_matching(cv::Mat intensity_map);

        std::vector<std::vector<std::vector<energy_type> > > m_energy_map;
        const period_type m_mind = 8;
        const int m_ndOctaves = 5;
        const int m_ndSamplesPerOctave = 30;
        const double m_dstep = std::pow(2.0, 1.0 / (double) m_ndSamplesPerOctave);
        const int m_nd = m_ndOctaves * m_ndSamplesPerOctave + 1;
        const int m_nc = (int) floor((double) m_mind * pow(m_dstep, m_nd)) + 1;

        std::vector<std::vector<int> > m_kStarts;
        std::vector<std::vector<int> > m_kEnds;
        // std::vector<double> m_period_scale(m_nd);

        std::vector<std::vector<int> > m_positive_pulse_start;
        std::vector<std::vector<int> > m_positive_pulse_end;
        std::vector<std::vector<int> > m_negative_pulse_start;
        std::vector<std::vector<int> > m_negative_pulse_end;
        std::vector<double> m_xcorr_a;
        std::vector<double> m_xcorr_b;

        std::vector<std::vector<double> > m_positive_pulse_centers;

        const int m_row_size = m_nc * m_nd;
        std::vector<energy_type> m_best_energy_by_row;


        period_type *m_periods;
        inline const phase_type get_real_phase(const phase_type phase, const period_type period) const {
            return phase;
            const phase_type half_band = (phase_type) floor(period) >> 1;
            // const phase_type real_phase = (abs(phase + half_band) % (phase_type) floor(period)) - half_band;
            const phase_type shifted_phase = phase + half_band;
            const uint period_width = (uint) floor(period);
            const phase_type real_shifted_phase = shifted_phase % period_width;
            const phase_type real_phase = real_shifted_phase - half_band;
            return real_phase;
        };
        const int m_image_width = 400;
        const int m_image_height = 300;

    private:
        data_type *m_dataset_ptr;
        size_t m_search_space_length;
        int *m_parabola_center;
        energy_type *m_intersection_points;
        std::vector<old_tuple_type> m_best_nodes;
        cv::Mat m_integral_image;
        // std::vector<period_type> m_period_map;

        double m_half_width;
        double m_period_scale_factor;

        const double m_positive_pulse_width = 1.28; // a0
        const double m_negative_pulse_width = 4.48; // b0

        const float m_f_low = 1.0;
        const energy_type m_maxD = 1.5;
        const double m_lambda_c = 0.5;
        const double m_lambda_d = 0.2;
        const energy_type m_maxz = (const energy_type) ((1.0 + (m_lambda_c + m_lambda_d) * (energy_type) m_nd * (energy_type) m_nd) / (m_lambda_c * m_lambda_d));
        const phase_type m_first_phase = -((const phase_type) (m_nc / 2));


        const double m_last_period = m_mind * std::pow(m_dstep, m_nd - 1);

        inline const double cumulative_sum(int v, int start);

        void distance_transform_phase(data_type *dataset_row_ptr) const;

        void distance_transform_period(data_type *dataset_row_ptr) const;

        void calculate_energy_integral(const std::vector<std::vector<std::vector<energy_type> > > &energy_map,
                                       const std::vector<energy_type> &max_by_row, size_t row_number,
                                       data_type *dataset_row_ptr) const;

        old_tuple_type find_row_max(data_type *&dataset_row_ptr, data_type *&pBestNode) const;

        void integrate_row(const std::vector<std::vector<std::vector<energy_type> > > &energy_map,
                           energy_type Dnrm, data_type *dataset_row_ptr) const;

        inline const period_idx_type period_min(const phase_type phase, const period_type *periods) const;
    };

    class Polyfit {
    private:

        void draw_crop(const double *polynomial, const double *perspective_factors, const cv::Mat &drawImg_, double x0,
                       int image_row_num);

        void plot_crop_points(std::string suffix);
        void plot_polys(const cv::Mat &inpImg_, const double *polynomial, const double *perspective_factors,
                        double poly_period, const cv::Mat drawImg_, int poly_origin_center, double poly_origin_period);

        int eval_poly(int image_row_num, int poly_idx);
        void plot_fitted_polys(std::string suffix);
        cv::Mat drawDenseOptFlow(const cv::Mat &flow, const cv::Mat &img, int step, cv::Scalar color, const cv::Mat &mask);
        void fit_poly_on_points(std::vector<crd_cpp::old_tuple_type> points);
        void add_noise_to_polys(double std);
        cv::Mat calculate_flow(const cv::Mat &new_frame);
        void calculate_poly_points();
        void fit_poly_on_image();
        double eval_poly_loss(const double *poly, const double *perspect, const double period, const int margin);
        void fit_central(const int max_useless_iterations, const int max_num_iterations, const double kRelativeStepSize);

        cv::Mat m_intensity_map;

        // initial guess
        double m_polynomial[5] = {0, 0, 0, 0, 0};
        double m_perspective_factors[8] = {.01, .01, .01, .01, .01, .01, .01, .01};
        double m_poly_period = 100;

        double m_best_polynomial[5] = {0, 0, 0, 0, 0};
        double m_best_perspective_factors[8] = {.01, .01, .01, .01, .01, .01, .01, .01};
        double m_best_poly_period = 100;

        std::vector<cv::Point2f> m_crop_row_points;
        cv::Mat m_spammable_img;
        void fit_poly_on_points();
        int m_image_center;
        int m_image_height;
        double m_mean;

    public:
        cv::Mat m_image;
        static const int eval_poly(int image_row_num, int poly_idx, const double m_polynomial[5],
                                   const double m_perspective_factors[8], const double* m_poly_period);
        static const double eval_poly_double(int image_row_num, int poly_idx, const double m_polynomial[5],
                                   const double m_perspective_factors[8], const double* m_poly_period);
        Polyfit(cv::Mat image, cv::Mat intensity_map, std::vector<crd_cpp::old_tuple_type> ground_truth, cv::Mat &out_img);
        void fit(cv::Mat new_frame);
        void add_noise();
    };
}
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

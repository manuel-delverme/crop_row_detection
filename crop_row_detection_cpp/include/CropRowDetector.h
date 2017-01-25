//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

typedef int phase_type;
typedef float period_type;
typedef unsigned short period_idx_type;
typedef double energy_type;
typedef std::pair<phase_type, period_type> old_tuple_type;
typedef std::pair<phase_type, size_t> tuple_type;

struct data_type{
    energy_type tuple_value;
    phase_type c;
    period_idx_type d;
};
class CropRowDetector{
    // int width;
public:

    CropRowDetector();

    void load(const cv::Mat& intensity);
    void teardown();
    double CrossCorrelation(int row_number, old_tuple_type template_var_param, double positive_pulse_width,
                            double negative_pulse_width, size_t image_width);

    std::vector<old_tuple_type> find_best_parameters(std::vector<std::map<old_tuple_type, double>> &energy_map);

    std::vector<old_tuple_type>
    template_matching(std::vector<std::map<old_tuple_type, double>> &energy_map, const cv::Mat &Intensity,
            std::map<period_type, std::vector<phase_type>> Xs, const double positive_pulse_width,
    const double negative_pulse_width, const size_t window_width);

    const period_type m_mind = 8;
    const int m_ndOctaves = 5;
    const int m_ndSamplesPerOctave = 70;
    const double m_dstep = std::pow(2.0, 1.0 / (double) m_ndSamplesPerOctave);
    const int m_nd = m_ndOctaves * m_ndSamplesPerOctave + 1;
    const int m_nc = (int) floor((double)m_mind * pow(m_dstep, m_nd)) + 1;

private:
    cv::Mat m_integral_image;
    // std::vector<period_type> m_period_map;

    double m_half_width;
    double m_period_scale_factor;

    const float m_f_low = 1.0;
    const energy_type m_maxD = 1.5;
    const float m_lambda_c = 0.5f;
    const float m_lambda_d = 0.2f;

    const double m_last_period = m_mind * std::pow(m_dstep, m_nd-1);

    double cumulative_sum(int v, int start);
    inline const phase_type get_real_phase(phase_type phase, const period_type period_) const {
        /*
        uint period = (unsigned int) period_;
        const unsigned int half_band = period >> 1;
        int shifted_phase = phase + half_band;

        const int mask = shifted_phase >> sizeof(int) * CHAR_BIT - 1;
        const uint abs_phase = (unsigned int)(shifted_phase ^ mask) - mask;
        const phase_type real_phase = (abs_phase  % period) - half_band;
        return real_phase;
         */
        const phase_type half_band = (phase_type) floor(period_)>>1;
        const phase_type real_phase = (uint) (abs(phase + half_band) % (uint) floor(period_)) - half_band;
        return real_phase;
    };

    int period_min(const phase_type phase, period_type* periods);

    data_type *m_dataset_ptr;
};
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

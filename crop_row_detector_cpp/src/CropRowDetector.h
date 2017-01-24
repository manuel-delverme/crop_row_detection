//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

typedef int phase_type;
typedef double period_type;
typedef std::pair<phase_type, period_type> old_tuple_type;
typedef std::pair<phase_type, size_t> tuple_type;

struct data_type{
    double B;
    double minBV;
    phase_type c;
    size_t d;
};
namespace std
{
    template<>
    struct hash<std::pair<phase_type, size_t>> {
        inline size_t operator()(const std::pair<phase_type, size_t> &p) const {
            const int m_max_d = 256;
            size_t h, h2;
            h = ((uint) (p.first + m_max_d / 2));
            h2 = (uint) ((uint) p.second) & 0x0000ffff;
            return h*m_max_d+h2;
        }
    };
}

class CropRowDetector{
    // int width;
public:

    CropRowDetector();

    void load(const cv::Mat& intensity);
    double CrossCorrelation(int row_number, old_tuple_type template_var_param, double positive_pulse_width,
                            double negative_pulse_width, size_t image_width);

    std::vector<old_tuple_type> find_best_parameters(std::vector<std::map<old_tuple_type, double>> &energy_map,
                                                const std::map<period_type, std::vector<phase_type>> &Xs);

    std::vector<old_tuple_type>
    template_matching(std::vector<std::map<old_tuple_type, double>> &energy_map, const cv::Mat &Intensity,
            std::map<period_type, std::vector<phase_type>> Xs, const double positive_pulse_width,
    const double negative_pulse_width, const size_t window_width);

    const period_type m_mind = 8;
    const uint m_ndOctaves = 5;
    const uint m_ndSamplesPerOctave = 70;
    const double m_dstep = std::pow(2.0, 1.0 / (double) m_ndSamplesPerOctave);
    const uint m_nd = m_ndOctaves * m_ndSamplesPerOctave + 1;
    const int m_nc = (int) floor((double)m_mind * pow(m_dstep, m_nd-1)) + 1;

private:
    cv::Mat m_integral_image;
    // std::vector<period_type> m_period_map;

    double m_half_width;
    double m_period_scale_factor;

    const float m_f_low = 1.0;
    const float m_maxD = 0.5;
    const float m_lambda_c = 0.5f;
    const float m_lambda_d = 0.2f;

    const double m_last_period = m_mind * std::pow(m_dstep, m_nd-1);

    double cumulative_sum(int v, int start);

    size_t period_min(const phase_type phase, std::vector<period_type> periods);
};
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

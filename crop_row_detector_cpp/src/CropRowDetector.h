//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

typedef int phase_type;
typedef float period_type;
typedef std::pair<phase_type, period_type> tuple_type;

struct data_type{
    double B;
    double minBV;
    phase_type c;
    period_type d;
};
template <class T>
inline void hash_pair(std::size_t& hash, const T& p){
    std::hash<T> hash_func;
    hash ^= hash_func(p) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
}

template<typename S, typename T>
struct std::hash< std::pair<S, T> >{
    inline size_t operator()(const std::pair<S,T>& p) const{
        size_t hash = 0;
        hash_pair(hash, p.first);
        hash_pair(hash, p.second);
        return hash;
    }
};

class CropRowDetector{
    // int width;
public:

    CropRowDetector();

    void load(const cv::Mat& intensity);
    double CrossCorrelation(int row_number, tuple_type template_var_param, double positive_pulse_width,
                            double negative_pulse_width, size_t image_width);

    std::vector<tuple_type> find_best_parameters(std::vector<std::map<tuple_type, double>> energy_map,
                                                 const std::map<period_type, std::vector<phase_type>> &Xs);

    std::vector<tuple_type>
    template_matching(std::vector<std::map<tuple_type, double>> &energy_map, const cv::Mat &Intensity,
            std::map<period_type, std::vector<phase_type>> Xs, const double positive_pulse_width,
    const double negative_pulse_width, const size_t window_width);

private:
    cv::Mat m_integral_image;
    // std::vector<period_type> m_period_map;

    double m_half_width;
    double m_period_scale_factor;

    const float m_maxD = 1.0;
    const float m_f_low = 1.0;
    const float m_lambda_c = 0.5f;
    const float m_lambda_d = 0.2f;
    const uint m_min_d = 8;
    const uint m_samples_per_octave = 8;
    const uint m_n_octaves = 5;
    const double m_d_step = std::pow(2.0, 1.0 / (double) m_samples_per_octave);

    double cumulative_sum(int v, int start);

    std::vector<period_type> periods_of(const std::map<period_type, std::vector<phase_type>> &phase, phase_type i1);
};
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

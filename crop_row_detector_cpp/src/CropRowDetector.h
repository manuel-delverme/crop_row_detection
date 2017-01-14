//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

typedef int phase_type;
typedef float period_type;
typedef std::pair<phase_type, period_type> tuple_type;

class CropRowDetector{
    // int width;
public:

    CropRowDetector();

    int negative_pulse_end, negative_pulse_start,
        positive_pulse_end, positive_pulse_start,
	positive_pixels, negative_pixels;
    double positive_correlation_value, negative_correlation_value;

    double half_width, period_scale_factor;
  
    void load(const cv::Mat& intensity);
    double CrossCorrelation(int row_number, tuple_type template_var_param, double positive_pulse_width,
                            double negative_pulse_width, int image_width);

    std::vector<tuple_type> find_best_parameters(const std::vector<std::map<tuple_type, double>> energy_map,
                                                 const std::map<period_type, std::vector<phase_type>> &Xs);

    std::vector<tuple_type> template_matching(std::vector<std::map<tuple_type, double>> &energy_map, const cv::Mat &Intensity,
                      std::map<period_type, std::vector<phase_type>> Xs, const double positive_pulse_width,
                      const double negative_pulse_width, const int window_width);

private:
    cv::Mat m_integral_image;
    std::vector<period_type> m_period_map;

    const float m_maxD = 1.0;
    const float m_f_low = 1.0;
    const float m_lambda_c = 0.5;
    const float m_lambda_d = 0.2;

    double cumulative_sum(int v, int start);

    size_t index_of_period(period_type period);

    std::vector<period_type> periods_of(const std::map<period_type, std::vector<phase_type>> &phase, phase_type i1);
};
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H


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
    cv::Mat detect(cv::Mat& intensity, cv::Mat& temp);

    int saturate(int val, int val_min, int val_max) {
            return std::max(std::min(val, val_max), val_min);
    }
    std::vector<std::pair<int, int>> template_matching(std::vector<std::map<std::pair<int, int>, double>>& energy_map,
            const cv::Mat& Intensity, std::vector<std::pair<int, int>>, double positive_pulse_width,
                                                       double negative_pulse_width, int window_width);
    
    // std::pair<int, int> CropRowDetector::find_optimal_x(std::vector<int> f);
    double CrossCorrelation(int row_number, std::pair<int, int> template_var_param, double positive_pulse_width,
                            double negative_pulse_width, int image_width);
private:
    cv::Mat m_integral_image;
    double cumulative_sum(int v, int start);
    std::vector<std::pair<int, int>> find_optimal_x(std::vector<std::map<std::pair<int, int>, double>> energy_map, bool X,
                                                    bool num_rows, bool x, double f_low, double D_max);
};
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

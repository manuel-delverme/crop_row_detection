//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H


class CropRowDetector{
    // int width;
public:
  
    int negative_pulse_end, negative_pulse_start,
        positive_pulse_end, positive_pulse_start,
	positive_pixels, negative_pixels;
    double positive_correlation_value, negative_correlation_value;

    double half_width, period_scale_factor;
  
  
   
    CropRowDetector(cv::Mat&);
    cv::Mat detect(cv::Mat& intensity, cv::Mat& temp);

    int saturate(int val, int val_min, int val_max) {
            return std::max(std::min(val, val_max), val_min);
    }
    std::vector<std::pair<int, int>> template_matching(const cv::Mat& Intensity,
                                                       const int& d_min, const int& n_samples_per_octave,const int& n_octaves,
                                                       const double& positive_pulse_width, const double& negative_pulse_width,
                                                       const int& window_width);
    
    // std::pair<int, int> CropRowDetector::find_optimal_x(std::vector<int> f);
    double CrossCorrelation(const int& row_number, const std::pair<int, int>& template_var_param, const double& positive_pulse_width,
                            const double& negative_pulse_width, const int& image_width);
private:
    cv::Mat m_integral_image;
    double inline cumulative_sum(const int& v, const int& start);
};
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

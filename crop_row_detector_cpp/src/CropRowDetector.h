//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H


class CropRowDetector{
    // int width;
public:
    CropRowDetector(cv::Mat const);
    cv::Mat detect(cv::Mat& intensity, cv::Mat& temp);

    int saturate(int val, int val_min, int val_max) {
            return std::max(std::min(val, val_max), val_min);
    }
    std::vector<std::pair<int, int>> template_matching(const cv::Mat& Intensity,
                                                       int d_min,int n_samples_per_octave,int n_octaves,
                                                       double positive_pulse_width, double negative_pulse_width,
                                                       int window_width, int center_of_image_row
    );
    // std::pair<int, int> CropRowDetector::find_optimal_x(std::vector<int> f);
    double CrossCorrelation(int row_number, std::pair<int, int> template_var_param, double positive_pulse_width,
                            double negative_pulse_width, int image_width, int center_of_row);
private:
    cv::Mat m_integral_image;
    double cumulative_sum(int v, int start, int end);
};
#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

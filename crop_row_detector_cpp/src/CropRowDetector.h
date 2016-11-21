//
// Created by noflip on 21/11/16.
//

#ifndef NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H
#define NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H


class CropRowDetector{
    // int width;
    public:
        CropRowDetector (int);
        // CropRowDetector(std::map<std::string, double>);
        cv::Mat detect(cv::Mat& intensity, cv::Mat& temp);

        int saturate(int val, int val_min, int val_max) {
            return std::max(std::min(val, val_max), val_min);
        }

        int cumulative_sum(cv::Mat I, int u) {
            // TODO: replace with integral image
            int sum = 0;
            for(int i=0; i < u-1; i++){
                sum += I.at<uchar>(0, i);
            }
            return sum;
        }



    std::vector<int> template_matching(
                cv::Mat &Intensity,
                int d_min, int n_samples_per_octave, int n_octaves,
                int positive_pulse_width, int negative_pulse_width,
                int window_width, int center_of_image_row
        );
        // std::pair<int, int> CropRowDetector::find_optimal_x(std::vector<int> f);
        int CrossCorrelation(
                cv::Mat I, std::pair<int, int> template_var_param, int positive_pulse_width,
                int negative_pulse_width, int image_width, int center_of_row
        );
};


#endif //NEW_CROP_ROW_DETECTION_CROPROWDETECTOR_H

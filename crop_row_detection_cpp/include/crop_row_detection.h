#include <CropRowDetector.h>

namespace crd_cpp {
    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<old_tuple_type> &match_results);

    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<tuple_type> &match_results);
}
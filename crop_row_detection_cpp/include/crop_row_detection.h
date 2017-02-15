#include <CropRowDetector.h>

namespace crd_cpp {
    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<old_tuple_type> &match_results);
    void dump_template_matching(std::vector<old_tuple_type> &match_results, int image_width, std::string prefix);

    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<tuple_type> &match_results);
}
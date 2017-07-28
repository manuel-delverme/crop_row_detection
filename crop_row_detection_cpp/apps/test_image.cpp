#include <opencv2/opencv.hpp>

#include <ctime>

#include <CropRowDetector.h>
#include <crop_row_detection.h>
#include <ImagePreprocessor.h>

#include <ceres/ceres.h>
#include "../src/PolyFit.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cout << "test_image <path/to/image>" << std::endl;
    }
    crd_cpp::CropRowDetector row_detector;
    clock_t start;

    cv::Size image_size = cv::Size(400, 300);

    std::cout << "loading" << std::endl;
    row_detector.pre_alloc(image_size);
    std::cout << "loaded" << std::endl;

    crd_cpp::ImagePreprocessor preprocessor(image_size);
    std::cout << "preprocessor initd" << std::endl;

    std::string image_file_name = argv[1];
    cv::Mat img = cv::imread(image_file_name);
    cv::resize(img, img, image_size);

    // cpp image processing
    cv::Mat intensityImage = preprocessor.process(img);

    /*
    // templte matching for cpp
    start = std::clock();
    row_detector.template_matching(intensityImage);
    std::cout << "template_matching time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    // optimization for cpp
    start = std::clock();
    std::cout << "optimization:" << std::endl;
    auto min_energy_results = row_detector.find_best_parameters(row_detector.m_energy_map,
                                                                row_detector.m_best_energy_by_row);
    std::cout << "best_param time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    // show results for CPP
    std::cout << "plotting" << std::endl;
    // dump cpp results
    // crd_cpp::dump_template_matching(min_energy_results, row_detector.m_image_width, "cpp");

    cv::Mat out_image;
    cv::cvtColor(intensityImage, out_image, cv::COLOR_GRAY2BGR);
    // crd_cpp::plot_template_matching(intensityImage, min_energy_results, out_image);

     */
    std::cout << ">fitting initial guess" << std::endl;
    crd_cpp::Polyfit polyfit(img, intensityImage, atoi(argv[2]), atoi(argv[3]), 1e-7);

    // teardown cpp
    std::cout << "teardown" << std::endl;
    row_detector.teardown();
    return 0;
}

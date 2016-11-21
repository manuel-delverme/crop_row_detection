// ros
// #include <ros/ros.h>
// #include <ros/subscriber.h>
// #include <sensor_msgs/Image.h>

// cv3
// #include <opencv3/highgui/highgui.hpp>
// #include <opencv3/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
#include "ImagePreprocessor.h"
#include "CropRowDetector.h"


void display_img(cv::Mat image){
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", image);
    cv::waitKey(0);
}

/*
 * FLOW:
 *  preprocess img
 *  - downsample
 *  - exg
 *  discretization
 *  - binization
 *  energy_eval
 *  -
 *  detector
 *      -- last piece
 *      -- postprocess smoothing
 *  -
 *  score
 */
int main(int argc, char** argv){
    if(argc != 2){
        return -1;
    }
    std::map<std::string, double> settings; // setup();
    settings["a0"] = 1.28;
    settings["b0"] = 4.48;

    std::cout << "initing preprocess" << std::endl;
    ImagePreprocessor preprocessor (argv[1], cv::Size(100,100));
    std::cout << "process" << std::endl;
    std::vector<cv::Mat> data = preprocessor.process();
    //TODO test results against paper-generated
    CropRowDetector row_detector (42);

    // vvv this is from cfg file
    int d_min = 8;
    int n_samples_per_octave = 70;
    int n_octaves = 5;
    int window_width = 10; // out of my ass

    for (cv::Mat& pIntensityImg : data) {
        // std::cout << "imshow" << std::endl;
        // display_img(pIntensityImg);
        // row_detector.detect(pIntensityImg, temp
        // int center, // uc
        std::cout << "parsing picture" << std::endl;
        std::vector<int> match_results = row_detector.template_matching(
                pIntensityImg, d_min, n_samples_per_octave,
                n_octaves, (int) settings["a0"], (int) settings["b0"],
                window_width, // w
                window_width / 2
        );

        // x_best = row_detector.find_optimal_x(f);
    }
    std::cout << "done" << std::endl;
    return 0;
}

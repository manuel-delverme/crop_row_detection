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

using namespace std;

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
    settings["width"] = 200;
    settings["height"] = 150;

    ImagePreprocessor preprocessor (argv[1], cv::Size((int) settings["width"], (int) settings["height"]));
    std::vector<cv::Mat> data = preprocessor.process();
    //TODO test results against paper-generated

    
    // vvv this is from cfg file
    int d_min = (8.f/400.f)*settings["width"];
    int n_samples_per_octave = 70;
    int n_octaves = 5;

    cv::Mat temp_image;

    for (cv::Mat& pIntensityImg : data) {
        // std::cout << "imshow" << std::endl;
        // display_img(pIntensityImg);
        // row_detector.detect(pIntensityImg, temp
        // int center, // uc
        // std::cout << "parsing picture" << std::endl;
        CropRowDetector row_detector(pIntensityImg); // TODO: split in constructor, and load
        std::vector<std::pair<int, int>> match_results = row_detector.template_matching(
                pIntensityImg, d_min, n_samples_per_octave,
                n_octaves, settings["a0"], settings["b0"],
                (int) settings["width"]);
        cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);

	cout << "dopo template matching" << endl;
	
        int image_height = pIntensityImg.size[0];
        int image_width = pIntensityImg.size[1];
        std::pair<int, int> x;

        for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
            x = match_results.at((unsigned long) image_row_num);
            // int column = x.first + (int) std::round(settings["width"]/2);
            int phase = x.first;
            int period = x.second;
            int center_of_image = (int) std::round(settings["width"] / 2);
            std::cout << image_row_num << " " << phase << "," << period << std::endl;
            int column = center_of_image + phase;
            while(column < image_width) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>(image_row_num, column);
                //std::cerr << ">drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column += period;
            }

            column = center_of_image + phase - period;
            while(column >= 0) {
                cv::Vec3b& pPixel = temp_image.at<cv::Vec3b>(image_row_num, column);
                // std::cerr << "<drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column -= period;
            }
        }
        display_img(temp_image);

        // x_best = row_detector.find_optimal_x(f);
    }
    std::cout << "done" << std::endl;
    return 0;
}

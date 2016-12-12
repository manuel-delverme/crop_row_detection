// ros
// #include <ros/ros.h>
// #include <ros/subscriber.h>
// #include <sensor_msgs/Image.h>

// cv3
// #include <opencv3/highgui/highgui.hpp>
// #include <opencv3/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>
// #include <opencv2/plot.hpp>
#include <fstream>
#include "ImagePreprocessor.h"
#include "CropRowDetector.h"

#define DEBUG 1

using namespace std;

vector<tuple_type> find_best_parameters(vector<map<tuple_type, double>> vector);

void display_img(cv::Mat image){
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", image);
    cv::waitKey(0);

    // close the window
    cv::destroyWindow("Display Image");
}

void plot_template_matching(const cv::Mat &pIntensityImg, vector<tuple_type> &match_results) {
    cv::Mat temp_image;
    cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);
    int image_height = pIntensityImg.size[0];
    int image_width = pIntensityImg.size[1];
    tuple_type x;

    for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
            x = match_results.at((unsigned long) image_row_num);
            // int column = x.first + (int) std::round(settings["width"]/2);
            phase_type phase = x.first;
            period_type period = x.second;
            int center_of_image = (int) ::std::round(image_width / 2);
            cout << image_row_num << " " << phase << "," << period << endl;
            int column = (int) (center_of_image + phase);
            while(column < image_width) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>(image_row_num, column);
                //std::cerr << ">drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column += period;
            }

            column = (int) (center_of_image + phase - period);
            while(column >= 0) {
                cv::Vec3b& pPixel = temp_image.at<cv::Vec3b>(image_row_num, column);
                // std::cerr << "<drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column -= period;
            }
        }
    display_img(temp_image);
}

map<period_type, vector<phase_type>> get_Xs(int d_min, int n_samples_per_octave, int n_octaves) {
    map<period_type, vector<phase_type>> Xs;
    vector<phase_type> phases;
    period_type period;

    int n_samples = (n_samples_per_octave * n_octaves);
    for (int sample_number = 0; sample_number <= n_samples; sample_number++) { // periods
        phases.clear();
        period = (period_type) (d_min * std::pow(2.f, (double) sample_number / (double) n_samples_per_octave));
        period = std::trunc(period * 100000) / 100000; // HAHAHHAHAHAHAH
        int half_band = (int) round(0.5 * period);

        for (int phase = -half_band; phase < half_band; phase++) {
            // x = std::make_pair(phase, period);
            phases.push_back(phase);
        }
        assert(phases.size() % 2 == 0);
        Xs[period] = phases;
    }
    return Xs;
}

vector<map<tuple_type, double>> load_match_results_from_csv(map<period_type, vector<phase_type>>& Xs) {
    vector<map<tuple_type, double>> energy_map;
    std::ifstream csv_stream("../energy_by_row.csv");
    std::string row;

    // drop header
    std::getline(csv_stream, row);
    std::cout << "read: " << row  << std::endl;

    int row_number;
    int old_row_number = -1;
    period_type old_period = -1;
    double score;
    phase_type phase;
    period_type period;
    tuple_type x;
    std::vector<phase_type> phases;

    while(std::getline(csv_stream, row)){
        std::stringstream ss(row);

        ss >> row_number;
        ss.ignore();
        if(old_row_number != row_number){
            old_row_number = row_number;
            energy_map.push_back(std::map<tuple_type, double>());
        }

        ss >> score;
        ss.ignore();

        ss >> period;
        ss.ignore();

        if(old_period != period){
            if(old_period != -1){
                Xs.insert(std::make_pair(old_period, phases));
            }

            old_period = period;
            phases.clear();
        }

        ss >> phase;
        x = std::make_pair(phase, period);
        phases.push_back(phase);
        energy_map.at(row_number)[x] = score;

    }
    return energy_map;
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
    settings["width"] = 400;
    settings["height"] = 300;
    // vvv this is from cfg file
    int d_min = 8;
    int n_samples_per_octave = 70;
    int n_octaves = 5;

    cv::Size image_size = cv::Size((int) settings["width"], (int) settings["height"]);

    ImagePreprocessor preprocessor (argv[1], image_size);

    CropRowDetector row_detector = CropRowDetector();
    map<period_type, vector<phase_type>> Xs;
#if !DEBUG
    Xs = get_Xs(d_min, n_samples_per_octave, n_octaves);
#endif
    std::vector<std::map<tuple_type, double>> energy_map((unsigned long) image_size.height);
    std::vector<cv::Mat> data = preprocessor.process();

    for (cv::Mat& pIntensityImg : data) {

#if DEBUG
        energy_map = load_match_results_from_csv(Xs);
#else
        row_detector.load(pIntensityImg);
        std::vector<std::pair<int, int>> match_results = row_detector.template_matching(energy_map, pIntensityImg, Xs,
                                                                                        settings["a0"], settings["b0"],
                                                                                        (int) settings["width"]);
        plot_template_matching(pIntensityImg, match_results);
#endif
        std::vector<tuple_type> min_energy_results = row_detector.find_best_parameters(energy_map, Xs);

    }
    std::cout << "done" << std::endl;
    return 0;
}

/*
vector<pair<int, int>> find_best_parameters(vector<map<pair<int, int>, double>> vector) {
    return vector<pair<int, int>>();
}
 */

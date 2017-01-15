#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>
#include "ImagePreprocessor.h"
#include "CropRowDetector.h"

#define DEBUG 1
void display_img(cv::Mat image){
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", image);
    cv::waitKey(0);

    // close the window
    cv::destroyWindow("Display Image");
}

void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<tuple_type> &match_results) {
    cv::Mat temp_image;
    cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);
    size_t image_height = (size_t) pIntensityImg.size[0];
    size_t image_width = (size_t) pIntensityImg.size[1];
    tuple_type x;

    for (size_t image_row_num = 0; image_row_num < image_height; image_row_num++) {
            x = match_results.at(image_row_num);
            phase_type phase = x.first;
            period_type period = x.second;
            period_type center_of_image = (period_type) std::round(image_width / 2);
            std::cout << image_row_num << " " << phase << "," << period << std::endl;
            period_type column = center_of_image + (period_type) phase;
            while(column < image_width) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num,(int) column);
                pPixel[2] = 255;
                column += period;
            }

            column = center_of_image + (period_type) phase - period;
            while(column >= 0) {
                cv::Vec3b& pPixel = temp_image.at<cv::Vec3b>((int) image_row_num,(int) column);
                // std::cerr << "<drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column -= period;
            }
        }
    display_img(temp_image);
}

std::map<period_type, std::vector<phase_type>> get_Xs(period_type d_min, uint n_samples_per_octave, uint n_octaves) {
    std::map<period_type, std::vector<phase_type>> Xs;
    std::vector<phase_type> phases;
    period_type period;

    uint n_samples = n_samples_per_octave * n_octaves;
    for (uint sample_number = 0; sample_number <= n_samples; sample_number++) { // periods
        phases.clear();
        period = (period_type) (d_min * std::pow(2.f, (double) sample_number / (double) n_samples_per_octave));
        period = std::trunc(period * 100000) / 100000;
        int half_band = (int) round(0.5 * period);

        for (int phase = -half_band; phase < half_band; phase++) {
            phases.push_back(phase);
        }
        assert(phases.size() % 2 == 0);
        Xs[period] = phases;
    }
    return Xs;
}

std::vector<std::map<tuple_type, double>> load_match_results_from_csv(std::map<period_type, std::vector<phase_type>>& Xs) {
    std::vector<std::map<tuple_type, double>> energy_map;
    std::ifstream csv_stream("../energy_by_row.csv");
    std::string row;

    // drop header
    std::getline(csv_stream, row);

    int row_number = - 1;
    int old_row_number = -1;
    period_type old_period = -1;
    double score;
    phase_type phase;
    period_type period;
    tuple_type x;
    std::vector<phase_type> phases;
    bool save_Xs = true;

    while(std::getline(csv_stream, row)){
        std::stringstream ss(row);

        ss >> row_number;
        ss.ignore();
        ss >> score;
        ss.ignore();
        ss >> period;
        ss.ignore();
        ss >> phase;

        if(old_period != period){ // this is a new period
            if(old_period != -1 && save_Xs){ // first time there is nothing to add
                Xs.insert(std::make_pair(old_period, phases)); // save the old period/phase
            }
            old_period = period;
            phases.clear();
        }

        if(old_row_number != row_number){ // row changed
            if(phases.size() > 0 && save_Xs) { // this happens if there is only one period
                Xs.insert(std::make_pair(old_period, phases));
            }
            if(old_row_number != -1){
                save_Xs = false;
            }
            old_row_number = row_number;
            energy_map.push_back(std::map<tuple_type, double>()); // add a new row to the structure
            phases.clear(); // just in case two rows have the same phase
        }

        x = std::make_pair(phase, period);
        phases.push_back(phase);
        energy_map.at(row_number)[x] = score;
    }
    if(save_Xs){
        // add the last row
        Xs.insert(std::make_pair(old_period, phases)); // save the old period/phase
    }
    return energy_map;
}

int main(int argc, char** argv){
    if(argc != 2){
        return -1;
    }
    std::map<std::string, double> settings; // setup();
    settings["a0"] = 1.28;
    settings["b0"] = 4.48;
    settings["width"] = 400;
    settings["height"] = 300;
    settings["d_min"] = 8;
    settings["n_samples_per_octave"] = 70;
    settings["n_octaves"] = 5;

    cv::Size image_size = cv::Size((uint) settings["width"], (uint) settings["height"]);
    ImagePreprocessor preprocessor (argv[1], image_size);

    CropRowDetector row_detector = CropRowDetector();
    std::map<period_type, std::vector<phase_type>> Xs;
#if !DEBUG
    Xs = get_Xs((period_type) settings["d_min"], (uint) settings["n_samples_per_octave"], (uint) settings["n_octaves"]);
#endif
    std::vector<std::map<tuple_type, double>> energy_map((size_t) image_size.height);
    std::vector<cv::Mat> data = preprocessor.process();

    for (cv::Mat& pIntensityImg : data) {

        std::clock_t start = std::clock();
#if DEBUG
        energy_map = load_match_results_from_csv(Xs);
#else
        row_detector.load(pIntensityImg);
        std::vector<tuple_type> match_results = row_detector.template_matching(energy_map, pIntensityImg, Xs, settings["a0"], settings["b0"], (size_t) settings["width"]);
#endif
        std::vector<tuple_type> min_energy_results = row_detector.find_best_parameters(energy_map, Xs);
        std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
#if !DEBUG
        // plot_template_matching(pIntensityImg, match_results);
#endif
        // plot_template_matching(pIntensityImg, min_energy_results);

    }
    std::cout << "done" << std::endl;
    return 0;
}

//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <opencv2/opencv.hpp>
// #include <opencv2/plot.hpp>

#include "CropRowDetector.h"

// MACRO per fixing del biased rounding
#define DOUBLE2INT(x)	(int)floor(x + 0.5)

CropRowDetector::CropRowDetector() {}

void CropRowDetector::load(cv::Mat const &intensity_map) {
    cv::Mat intensity_map_64f;
    intensity_map.convertTo(intensity_map_64f, CV_64F);
    m_integral_image = cv::Mat::zeros( intensity_map.size(), CV_64F);

    for(int row = 0; row < intensity_map.rows; row++) {
        for (int column = 1; column < intensity_map.cols; column++){
            m_integral_image.at<double>(row,column) = (double)intensity_map.at<uchar>(row,column) + m_integral_image.at<double>(row,column-1);
        }
    }
    period_scale_factor = .125;
    half_width = intensity_map.cols/2;

}

/*cv::Mat CropRowDetector::detect(cv::Mat& intensity, cv::Mat& templ){
    // Mat templ; // from settings
    cv::Mat result;
    int result_rows = intensity.rows - templ.rows;
    int result_cols = intensity.cols - templ.cols;
    result.create(result_rows, result_cols, CV_8UC1);

    cv::Mat average_intensity;
    average_intensity.create(1, intensity .cols, CV_64F);

    cv::Mat plot_image;
    plot_image.create(1, intensity.cols, CV_64F);

    cv::Ptr<cv::plot::Plot2d> plot;
    cv::Mat plot_result;
    for (int i=intensity.rows - 1; i >= intensity.rows / 2 ; i--) {
        intensity.row(i).convertTo(plot_image, CV_64F);
        average_intensity += (plot_image / intensity.rows);
    }
    std::cout << "displaying avg" << std::endl;
    plot = cv::plot::createPlot2d(average_intensity);
    plot->render(plot_result);
    cv::imshow("row intensity", plot_result);
    cv::waitKey(0);
    return result;
}*/

std::vector<std::pair<int, int>> CropRowDetector::template_matching(
        std::vector<std::map<std::pair<int, int>, double>>& energy_map,
        const cv::Mat& Intensity,
        // const std::vector<std::pair<int, int>> Xs,
        std::map<uint, std::vector<int>> Xs,
        const double positive_pulse_width,
        const double negative_pulse_width,
        const int window_width // w
) {
    int image_height = Intensity.size[0];

    double energy = 0;
    double best_energy = -1;

    std::pair<int, int> best_pair;
    std::vector<std::pair<int,int>> best_pairs((unsigned long) image_height);
    std::map<std::pair<int,int>, double> row_energy_map;

    for (int image_row_num = 0; image_row_num < image_height; image_row_num++) {
        row_energy_map.clear();
        for(std::pair<uint, std::vector<int>> const& item: Xs){
            uint period = item.first;
            std::vector<int> phases = item.second;
            for(int phase: phases) {
                std::pair<uint, int> x = std::make_pair(phase, period);

                energy = CrossCorrelation(image_row_num, x, positive_pulse_width, negative_pulse_width, window_width);
                row_energy_map[x] = energy;
                if (energy > best_energy) {
                    best_energy = energy;
                    best_pair = x;
                }
            }
        }
        energy_map.push_back(row_energy_map);
        best_pairs[image_row_num] = best_pair;
        best_energy = -1;
    }
    return best_pairs;
}
double CropRowDetector::CrossCorrelation(int row_number, std::pair<int, int> template_var_param,
                                         double positive_pulse_width, double negative_pulse_width,
                                         int image_width){

    double phase = template_var_param.first;
    double period = template_var_param.second;

    // Calcolo quantità necesarrie a CrossCorrelation
    double scale = period*period_scale_factor;

    int a = DOUBLE2INT(scale*positive_pulse_width);
    int b = DOUBLE2INT(scale*negative_pulse_width);

    double halfb = .5*b;
    double halfa = .5*a;
    double halfd = .5*period;

    int kStart = (int)floor((-(image_width / 2 + phase) + halfa) / period);    // offset prima onda
    int kEnd = (int)floor(((image_width / 2 - phase) + halfa) / period);   // offset ultima onda prima della fine dell'immagine


    // Calcolo centro dell'onda quadra positiva e negativa
    double distance_positive_negative_pulse_center =  halfd - halfb;
    double positive_pulse_center = (double)phase + (double)kStart * (double)period + half_width;


    // Calcolo per Onda ad inizio immagine, non calcolabile con la classica routine

    positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);
    positive_pulse_end = positive_pulse_start + a - 1;

    negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
    negative_pulse_end = negative_pulse_start + b - 1;

    //cout << positive_pulse_center <<  " " << halfa << " " << positive_pulse_start << " " << positive_pulse_end << " " << negative_pulse_start << " " << negative_pulse_end << endl;

    if(positive_pulse_end >= 0) {
        positive_correlation_value = cumulative_sum(row_number, positive_pulse_end);
        positive_pixels = positive_pulse_end;
    }
    else {
        positive_correlation_value = 0;
        positive_pixels = 0;
    }


    if(negative_pulse_start < 0)
        negative_pulse_start = 0;

    if(negative_pulse_end >= 0) {
        negative_correlation_value = cumulative_sum(row_number, negative_pulse_end)
                                     -cumulative_sum(row_number, negative_pulse_start); //tolto  -cumulative_sum(row_number, negative_pulse_start-1);

        negative_pixels = negative_pulse_end - negative_pulse_start + 1;
    }
    else {
        negative_correlation_value = 0;
        negative_pixels = 0;
    }


    positive_pulse_center += period;

    for(int k = kStart + 1; k < kEnd; k++, positive_pulse_center += period)	{




        positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);
        positive_pulse_end = positive_pulse_start + a - 1;

        positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                      - cumulative_sum(row_number, positive_pulse_start-1);

        positive_pixels += (positive_pulse_end - positive_pulse_start + 1);

        negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
        negative_pulse_end = negative_pulse_start + b - 1;

        negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                      -cumulative_sum(row_number, negative_pulse_start-1);

        negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
    }

    positive_pulse_start = (int) std::floor(positive_pulse_center - halfa);

    positive_pulse_end = positive_pulse_start + a - 1;

    if(positive_pulse_end >= image_width)
        positive_pulse_end = image_width - 1;

    positive_correlation_value += cumulative_sum(row_number, positive_pulse_end)
                                  -cumulative_sum(row_number, positive_pulse_start-1);

    positive_pixels += (positive_pulse_end - positive_pulse_start + 1);


    negative_pulse_start = (int) std::floor(positive_pulse_center + distance_positive_negative_pulse_center);
    if(negative_pulse_start < image_width)
    {
        negative_pulse_end = negative_pulse_start + b - 1;

        if(negative_pulse_end >= image_width)
            negative_pulse_end = image_width - 1;

        negative_correlation_value += cumulative_sum(row_number, negative_pulse_end)
                                      -cumulative_sum(row_number, negative_pulse_start-1);

        negative_pixels += (negative_pulse_end - negative_pulse_start + 1);
    }

    return (negative_pixels * positive_correlation_value - positive_pixels * negative_correlation_value) / (double)(positive_pixels * negative_pixels);
}



double CropRowDetector::cumulative_sum(int v, int start) {
    // TODO turn in start-end
    return m_integral_image.at<double>(v, start);
}

std::vector<tuple_type> CropRowDetector::find_best_parameters(const std::vector<std::map<tuple_type, double>> energy_map,
                                      std::map<period_type, std::vector<phase_type >> Xs) {
    uint image_height = (uint) energy_map.size();

    // TODO to be defined with real variables
    float lambda_c = 0.5;
    float lambda_d = 0.5;

    std::vector<phase_type> phase_parab_center(Xs.size());
    std::vector<period_type> period_parab_center(Xs[0].size());

    std::vector<std::map<tuple_type, double>> minBV = energy_map;

    tuple_type best_node;
    std::map<tuple_type, double> B;
    std::vector<tuple_type> best_nodes;

    for (uint row_number = 0; row_number < image_height; row_number++) {

        uint parabola_index;

        int maxz = INT_MAX;
        int minz = INT_MIN;

        std::vector<phase_type> phase_parabola_mean(Xs.size());
        std::vector<phase_type> phase_intersection_points(Xs.size());

        double fq,  q2,  fvk,  vk2;
        phase_type q;

        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;
            phase_type phase_range_offset = (phase_type) std::floor(phases.size() / 2);

            parabola_index = 0;

            double left_term;
            double right_term;
            double normalizer;
            phase_type last_intersection;

            phase_parabola_mean[0] = 0;
            phase_intersection_points[0] = minz; // -inf
            phase_intersection_points[1] = maxz; // +inf

            // for the second parabola until the last
            for (auto phase_iter=phases.begin() + 1; phase_iter != phases.end(); phase_iter++)
            {
                phase_type phase = *phase_iter;
                tuple_type x = std::make_pair(phase, period);
                assert((phase + phase_range_offset) > 0);

                q = phase + phase_range_offset;
                while (true) {
                    fq = energy_map.at(row_number).at(x);
                    q2 = lambda_c * std::pow(q, 2);

                    tuple_type x_vk = std::make_pair(phase_parabola_mean[parabola_index] - phase_range_offset, period);
                    fvk = energy_map.at(row_number).at(x_vk);

                    vk2 = lambda_c * std::pow(phase_parabola_mean[parabola_index], 2);
                    normalizer = 2.0 * lambda_c * (q - phase_parabola_mean[parabola_index]);

                    last_intersection = (phase_type) (((fq + q2) - (fvk + vk2) ) / normalizer);

                    if (last_intersection <= phase_intersection_points[parabola_index]){
                        parabola_index--; // overwrite the last one
                    } else {
                        break;
                    }
                }
                parabola_index++;
                // record the last intersection
                phase_parabola_mean[parabola_index] = q;
                phase_intersection_points[parabola_index] = last_intersection;

                // reinit for the next parabola
                phase_intersection_points[parabola_index + 1] = maxz;
            }

            parabola_index = 0; // 615

            double best_cost;
            double distance;
            // select the best points
            for(phase_type phase: phases){
                assert(phase_range_offset + phase >= 0);
                q = phase + phase_range_offset;

                while (phase_intersection_points[parabola_index + 1] < q) {
                    parabola_index++;
                }
                tuple_type x = std::make_pair(phase, period);
                distance = q - phase_parabola_mean[parabola_index];

                tuple_type x_prime = std::make_pair(phase_parabola_mean[parabola_index] - phase_range_offset, period);
                best_cost = minBV.at(row_number)[x_prime] + lambda_c * std::pow(distance, 2);
                minBV.at(row_number).insert(std::make_pair(x, best_cost));
            }
        } // 633
        maxz = INT_MAX;

        std::vector<period_type> period_intersection_points;
        phase_parabola_mean.clear(); // TODO: needded?

        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs) {
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;

            period_type last_intersection;
            double left_term, right_term, normalizer;

            for (auto phase_iter = phases.begin() + 1; phase_iter != phases.end(); phase_iter++) {
                phase_type phase = *phase_iter;
                tuple_type x = std::make_pair(phase, period);

                parabola_index = 0;
                phase_parabola_mean[0] = 0;
                period_parab_center[0] = Xs.begin()->first;
                period_intersection_points[0] = -maxz;
                period_intersection_points[1] = maxz;

                tuple_type x_prime;
                while (true) {
                    // TODO: check math
                    x_prime = std::make_pair(0, phase_parabola_mean[parabola_index]);

                    left_term = minBV.at(row_number)[x] + lambda_d * std::pow(period, 2);
                    right_term =
                            minBV.at(row_number)[x_prime] + lambda_d * std::pow(phase_parab_center[parabola_index], 2);

                    normalizer = (2.0 * lambda_d * (period - phase_parab_center[parabola_index]));
                    last_intersection = (period_type) ((left_term - right_term) / normalizer);

                    if (last_intersection <= phase_intersection_points[parabola_index])
                        parabola_index--;
                    else
                        break;
                }
                parabola_index++;

                phase_parab_center[parabola_index] = phase;
                period_parab_center[parabola_index] = period;

                period_intersection_points[parabola_index] = last_intersection;
                period_intersection_points[parabola_index + 1] = maxz;
            }
            parabola_index = 0;
            // reset phase
        }
        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;
            phase_type phase_range_offset = (phase_type) std::floor(phases.size() / 2);

            for(phase_type phase: phases) {

                while (phase_intersection_points[parabola_index + 1] < period) {
                    parabola_index++;
                }
                tuple_type x = std::make_pair(phase + phase_range_offset, period);
                tuple_type x_prime = std::make_pair(period_parab_center[parabola_index] + phase_range_offset, period);
                period_type iTmp = period - phase_parabola_mean[parabola_index];

                minBV.at(row_number)[x] = minBV.at(row_number)[x_prime] + lambda_d * std::pow(iTmp, 2);
            }
        }
        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs){
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;

            for(phase_type phase: phases) {
                tuple_type x = std::make_pair(phase, period);

                if(B[x] < B[best_node])
                {
                    best_node = x;
                }
            }
        }
        best_nodes[row_number] = best_node;
    }

    for(uint row_number = image_height - 2; row_number >= 0; row_number--)
    {
        // lol
    }
}

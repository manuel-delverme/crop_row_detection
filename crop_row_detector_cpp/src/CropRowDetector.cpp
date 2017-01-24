//
// Created by noflip on 21/11/16.
//

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <unordered_map>
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
    m_period_scale_factor = .125;
    m_half_width = intensity_map.cols/2;

}

std::vector<old_tuple_type> CropRowDetector::template_matching(
        std::vector<std::map<old_tuple_type, double>> & energy_map,
        const cv::Mat& Intensity,
        // const std::vector<std::pair<int, int>> Xs,
        std::map<period_type, std::vector<phase_type>> Xs,
        const double positive_pulse_width,
        const double negative_pulse_width,
        const size_t window_width // w
) {
    size_t image_height = (size_t) Intensity.size[0];

    double energy = 0;
    double best_energy = -1;

    old_tuple_type best_pair;
    std::vector<old_tuple_type> best_pairs(image_height);
    std::map<old_tuple_type, double> row_energy_map;

    for (size_t image_row_num = 0; image_row_num < image_height; image_row_num++) {
        row_energy_map.clear();
        // size_t period_idx = 0;
        for(std::pair<period_type, std::vector<phase_type>> const& item: Xs) {
            period_type period = item.first;
            std::vector<phase_type> phases = item.second;
            for(const phase_type phase: phases) {
                old_tuple_type x = std::make_pair(phase, period);

                energy = CrossCorrelation((int) image_row_num, x, positive_pulse_width, negative_pulse_width, window_width);
                row_energy_map[x] = energy;
                if (energy > best_energy) {
                    best_energy = energy;
                    best_pair = x;
                }
            }
        }
        energy_map.at(image_row_num) = row_energy_map;
        // TODO: extract best energy value Dnrm for later processing
        best_pairs[image_row_num] = best_pair;
        best_energy = -1;
    }
    return best_pairs;
}
double CropRowDetector::CrossCorrelation(int row_number, old_tuple_type template_var_param,
                                         double positive_pulse_width, double negative_pulse_width,
                                         size_t image_width){

    phase_type phase = template_var_param.first;
    period_type period = template_var_param.second;

    int negative_pulse_end, negative_pulse_start, positive_pulse_end, positive_pulse_start, positive_pixels, negative_pixels;
    double positive_correlation_value, negative_correlation_value;


    // Calcolo quantità necesarrie a CrossCorrelation
    double scale = period*m_period_scale_factor;

    int a = DOUBLE2INT(scale*positive_pulse_width);
    int b = DOUBLE2INT(scale*negative_pulse_width);

    double halfb = .5*b;
    double halfa = .5*a;
    double halfd = .5*period;

    int kStart = (int)floor((-((int) image_width / 2 + phase) + halfa) / period);    // offset prima onda
    int kEnd = (int)floor((((int) image_width / 2 - phase) + halfa) / period);   // offset ultima onda prima della fine dell'immagine


    // Calcolo centro dell'onda quadra positiva e negativa
    double distance_positive_negative_pulse_center =  halfd - halfb;
    double positive_pulse_center = (double)phase + (double)kStart * (double)period + m_half_width;


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
            negative_pulse_end = (int) (image_width - 1);

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

std::vector<old_tuple_type> CropRowDetector::find_best_parameters(std::vector<std::map<old_tuple_type, double>>& energy_map,
                                      const std::map<period_type, std::vector<phase_type >>& Xs) {

    // TODO:
    // should initialize all the unused phases (most of them for low periods)
    // to +inf and ignore them if required
    size_t image_height = (size_t) energy_map.size();

    // std::vector<std::map<tuple_type, data_type>> dataset(image_height);

    // TODO: reserve space for hashmaps
    data_type* dataset = new data_type[image_height * m_nd * m_nc];

    std::vector<old_tuple_type> best_nodes(image_height);

    auto comparer = [] (const auto & p1, const auto & p2) { return p1.second < p2.second;};
    double Dnrm;
    std::vector<phase_type> phases;

    int k;
    // size_t length = (size_t) Xs.rbegin()->first;
    size_t length = (size_t) std::max(Xs.size(), (size_t) Xs.rbegin()->first);

    // phase_type* v_ = new phase_type[length];
    std::vector<period_type> v__(length);
    size_t* parabola_centers_period_idx = new size_t[length];
    double *intersection_points = new double[length+1];
    // std::vector<double> z(length+1);
    std::vector<double> z(length+1);
    std::vector<phase_type> v_(length);

    double numerator;
    double b_val;

    std::vector<period_type> periods((size_t) m_nd);
    period_type period = m_mind;

    data_type *DP_ = dataset;
    data_type *pDP;
    data_type *DP__;
    double s;
    size_t id;
    phase_type c;
    double d;
    int n = m_nc * m_nd;

    for (size_t period_idx = 0; period_idx < m_nd; period_idx++, period *= m_dstep) {
        periods[period_idx] = period;
    }

    for (size_t row_number = 0; row_number < image_height; row_number++) {
        Dnrm = std::max_element(std::begin(energy_map.at(row_number)), std::end(energy_map.at(row_number)), comparer)->second; //TODO: take it from template_matching

        size_t period_idx = 0;
        DP__ = DP_;        // DP__ - ptr. to the first data in a block row

        for(id = 0; id < m_nd; id++, DP__ += m_nc) {
            pDP = DP__;

            for (c = -m_nc / 2; c < m_nc / 2; c++, pDP++) {
                const period_type period_ = periods[period_idx];
                const phase_type half_band = (phase_type) std::floor(0.5 * period_);
                const phase_type real_phase = (std::abs(c + half_band) % (int) std::floor(period_)) - half_band;
                const old_tuple_type x_energy = std::make_pair(real_phase, period_);

                auto D = energy_map.at(row_number).at(x_energy);
                if (Dnrm >= 1.0) {
                    pDP->B = 1.0 - D / Dnrm;

                    if (pDP->B > m_maxD)
                        pDP->B = m_maxD;
                } else{
                    pDP->B = m_maxD;
                }
                if (row_number > 0){
                    std::cout << c << " " << period_ << std::endl;
                    pDP->B += (pDP - n)->minBV;
                }
            }
        }
        if(row_number < image_height - 1) {
            // #pragma omp parallel for default(none) private(period, k, numerator) shared(periods, dataset, row_number, length)
            DP__ = DP_;
            double maxz;
            maxz = (1.0 + m_lambda_c * (double)(m_nc * m_nc)) / (2.0 * m_lambda_c);
            for(id = 0; id < m_nd; id++, DP__ += m_nc)
            {
                // period = periods[period_idx];
                k = 0;
                v_[0] = -m_nc/2;
                z[0] = -maxz;
                z[1] = +maxz;
                pDP = DP__ + 1;

                for(c = -m_nc/2 + 1; c < m_nc/2; c++, pDP++)
                {
                    while(true)
                    {
                        s = ((pDP->B + m_lambda_c * (double)(c * c)) - (DP__[v_[k] + m_nc / 2].B + m_lambda_c * (double)(v_[k] * v_[k]))) /
                            (2.0 * m_lambda_c * (double)(c - v_[k]));

                        if(s <= z[k])
                            k--;
                        else
                            break;
                    }

                    k++;

                    v_[k] = c;
                    z[k] = s;
                    z[k + 1] = maxz;
                }
                k = 0;
                pDP = DP__;

                for(c = -m_nc / 2; c < m_nc / 2; c++, pDP++)
                {
                    while(z[k + 1] < (double)c)
                    {
                        k++;
                    }
                    const double phase_dist = c - v_[k];
                    pDP->minBV = DP__[v_[k] + m_nc / 2].B + m_lambda_c * phase_dist * phase_dist;
                    pDP->c = v_[k];
                    pDP->d = (size_t) id;
                    //cout << v_[k] + m_nc / 2 << " " << DP__[v_[k] + m_nc / 2].B << " " << pDP->minBV << endl;
                }
            }

            // ----------------------------------------------------------------------------------------------------
#if DEBUG
            std::fstream outfile; outfile.open("/tmp/results_mio_phase", std::fstream::out);
            for (size_t period_idx = 0; period_idx < m_nd; ++period_idx) {
                period_type period = periods[period_idx];
                phase_type half_band = (phase_type) std::floor(0.5 * period);

                for (phase_type phase = -129; phase < -half_band + 1; phase++)
                    outfile << row_number << "," << period_idx << "," << phase << ",COCKS,COCKS" << std::endl;

                for (phase_type phase = -half_band + 1; phase < half_band; phase++) {
                    auto x = &dataset.at(row_number).find(std::make_pair(phase, period_idx))->second;
                    outfile << row_number << "," << period_idx << "," << phase << "," << x->B << "," << x->minBV << std::endl;
                }
                for (phase_type phase = half_band; phase < 129; phase++)
                    outfile << row_number << "," << period_idx << "," << phase << ",COCKS,COCKS" << std::endl;

            }
            outfile.close();
#endif
            // ----------------------------------------------------------------------------------------------------

            // std::cout << "done; working on periods" << std::endl;
            maxz = (1.0 + m_lambda_d * (double)(m_nd * m_nd)) / (2.0 * m_lambda_d);
            DP__ = DP_;

            for(c = 0; c < m_nc; c++, DP__++)
            {
                k = 0;
                v_[0] = 0; //for saving id
                v__[0] = (double)m_mind; //for saving d
                z[0] = -maxz;
                z[1] = maxz;

                pDP = DP__ + m_nc;

                d = (double)m_mind;
                for(id = 1; id < m_nd; id++, pDP += m_nc)
                {
                    d *= m_dstep;
                    while(true)
                    {

                        s = ((pDP->minBV + m_lambda_d * d * d) - (DP__[m_nc * v_[k]].minBV + m_lambda_d * v__[k] * v__[k])) /
                            (2.0 * m_lambda_d * (d - v__[k]));


                        std::cout << z[k] << " " << k  << " " << DP__[m_nc * v_[k]].minBV << " " << id << std::endl;
                        if(s <= z[k])
                            k--;
                        else
                            break;
                    }
                    k++;
                    v_[k] = id;
                    v__[k] = d;

                    z[k] = s;
                    z[k + 1] = maxz;
                }

                k = 0;

                pDP = DP__;

                d = (double)m_mind;
                for(id = 0; id < m_nd; id++, pDP += m_nc)
                {
                    while(z[k + 1] < d)	{
                        k++;
                    }
                    //iTmp = (id - v_[k]);
                    const double period_dist = d - v__[k];

                    pDP->minBV = DP__[m_nc * v_[k]].minBV + m_lambda_d * period_dist * period_dist;
                    pDP->c = DP__[m_nc * v_[k]].c;
                    pDP->d = (size_t) v_[k];

                    d *= m_dstep;

                }
            }
        }
    }
    DP_ = dataset + (image_height - 1) * n;
    DP__ = DP_;
    size_t row_number = image_height - 1;

    data_type* pBestNode = DP__;

    data_type *ppp = DP__;
    for(size_t iter = 0; iter < m_nc*m_nd; iter++, ppp++)
        std::cout << iter << " " << ppp->B << std::endl;

    phase_type bestc;
    size_t bestid;
    period_type bestd;

    d = (double)m_mind;
    for(id = 0; id < m_nd; id++, DP__ += m_nc, d *= m_dstep)
    {
        pDP = DP__;
        for(c = -m_nc/2; c < m_nc/2; c++, pDP++)
            if(pDP->B < pBestNode->B)
            {
                pBestNode = pDP;
                bestc = c;
                bestid = id;
                bestd = (period_type) d;
            }
    }
    best_nodes.at(row_number) = std::make_pair(bestc, bestd);
    // m_c[v] = bestc;
    // m_id[v] = bestid;
    // m_d[v] = bestd;
    // data_type best_node, node;


    while(row_number-- > 0) {
        pDP = pBestNode - n;

        DP_ -= n;

        pBestNode = DP_ + pDP->d * m_nc + m_nc / 2 + pDP->c;

        bestc = pDP->c;
        bestid = pDP->d;
        bestd = periods[bestd];

        best_nodes.at(row_number) = std::make_pair(bestc, periods[bestd]);
    }
    return best_nodes;
}

size_t CropRowDetector::period_min(const phase_type phase, std::vector<period_type> periods) {
    period_type period_min;
    if(phase < 0){
        period_min = (2 * -phase);
    } else {
        period_min = (2 * (phase + 1));
    }
    size_t period_idx;
    // for (period = m_mind; period < period_min; period *= m_dstep);
    for (period_idx = 0; periods[period_idx] < period_min; period_idx++);
    return period_idx;
}

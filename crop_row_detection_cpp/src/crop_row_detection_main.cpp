#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>
#include "ImagePreprocessor.h"
#include "ceres/ceres.h"

namespace crd_cpp {
    // using ceres::AutoDiffCostFunction;
    // using ceres::CostFunction;
    // using ceres::CauchyLoss;
    // using ceres::Problem;
    // using ceres::Solver;
    // using ceres::Solve;

    struct Residual {
        Residual(double x, double y, double x0, double frequency_factor) : x_(x), y_(y), x0_(x0), frequency_factor_(frequency_factor) {}
        template <typename T>
        bool operator()(const T* p, const T* pf, const T* f, T* residual) const {

            //funzione residuo da minimizzare
            residual[0] = T(y_) - ((pf[1] + T(x0_)*pf[0])*p[0]*pow(T(x_),4)+ (pf[3] + T(x0_)*pf[2])*p[1]*pow(T(x_),3)+
                                   (pf[5] + T(x0_)*pf[4])*p[2]*pow(T(x_),2)+ (pf[7] + T(x0_)*pf[6])*p[3]*pow(T(x_),1)+
                                   p[4]+T(frequency_factor_)*f[0]);
            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(double x, double y, double x0, double frequency_factor) {
            return (new ceres::AutoDiffCostFunction<Residual, 1,5,8,1>(new Residual(x, y, x0, frequency_factor)));
        }
        double x_;
        double y_;
        double x0_;
        double frequency_factor_;
    };

    void display_img(cv::Mat image) {
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", image);
        cv::waitKey(0);
        cv::destroyWindow("Display Image");
    }

    void dump_template_matching(std::vector<old_tuple_type> &match_results, int image_width, std::string prefix) {
        old_tuple_type x;
        std::cout << "dumping " << match_results.size() << " results" << std::endl;

        std::ofstream csv_stream("/tmp/match_results" + prefix + ".csv");

        period_type center_crop;
        period_type left_crop;
        period_type right_crop;
        for (size_t image_row_num = 0; image_row_num < match_results.size(); image_row_num++) {
            x = match_results.at(image_row_num);
            phase_type phase = x.first;
            period_type period = x.second;

            period_type center_of_image = (period_type) std::round(image_width / 2);

            center_crop = center_of_image + (period_type) phase;
            left_crop = center_crop - period;
            right_crop = center_crop + period;

            // std::cout
            //         << image_row_num << ", "
            //         << left_crop << ", "
            //         << center_crop << ", "
            //         << right_crop << std::endl;
            csv_stream
                    << left_crop << ", "
                    << center_crop << ", "
                    << right_crop << std::endl;

        }
        csv_stream.close();
    }

    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<old_tuple_type> &match_results) {
        cv::Mat temp_image;
        cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);
        size_t image_height = (size_t) pIntensityImg.size[0];
        size_t image_width = (size_t) pIntensityImg.size[1];
        old_tuple_type x;
        std::cout << "plotting " << match_results.size() << " results" << std::endl;

        for (size_t image_row_num = 0; image_row_num < image_height; image_row_num++) {
            x = match_results.at(image_row_num);
            phase_type phase = x.first;
            period_type period = x.second;

            period_type center_of_image = (period_type) std::round(image_width / 2);
            // std::cout << image_row_num << " " << phase << "," << period << std::endl;
            period_type column = center_of_image + (period_type) phase;
            while (column < image_width) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                pPixel[2] = 255;
                column += period;
            }

            column = center_of_image + (period_type) phase - period;
            while (column >= 0) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                pPixel[2] = 255;
                column -= period;
            }
        }
        display_img(temp_image);
    }


    void plot_template_matching(const cv::Mat &pIntensityImg, std::vector<tuple_type> &match_results) {
        cv::Mat temp_image;
        cv::cvtColor(pIntensityImg, temp_image, cv::COLOR_GRAY2BGR);
        size_t image_height = (size_t) pIntensityImg.size[0];
        size_t image_width = (size_t) pIntensityImg.size[1];
        old_tuple_type x;

        for (size_t image_row_num = 0; image_row_num < image_height; image_row_num++) {
            x = match_results.at(image_row_num);
            phase_type phase = x.first;
            period_type period = x.second;
            period_type center_of_image = (period_type) std::round(image_width / 2);
            // std::cout << image_row_num << " " << phase << "," << period << std::endl;
            period_type column = center_of_image + (period_type) phase;
            while (column < image_width) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                pPixel[2] = 255;
                column += period;
            }

            column = center_of_image + (period_type) phase - period;
            while (column >= 0) {
                cv::Vec3b &pPixel = temp_image.at<cv::Vec3b>((int) image_row_num, (int) column);
                // std::cerr << "<drawing on: " << column << std::endl;
                pPixel[2] = 255;
                column -= period;
            }
        }
        display_img(temp_image);
    }

    double eval_poly(double polynomial[5], double perspective_factors[8], unsigned int r, double x0){
        return (perspective_factors[1] + x0*perspective_factors[0]) * polynomial[0] * pow(r, 4)
               + (perspective_factors[3] + x0*perspective_factors[2]) * polynomial[1] * pow(r, 3)
               + (perspective_factors[5] + x0*perspective_factors[4]) * polynomial[2] * pow(r, 2)
               + (perspective_factors[7] + x0*perspective_factors[6]) * polynomial[3] * r
               + polynomial[4];


    }
    void polyfit(cv::Mat &inpImg_, std::vector<old_tuple_type> ground_truth){

        /* INIZIO PARAMETRIZZAZIONE */
        double polynomial[5] = {0, 0, 0, 0, 0};
        double perspective_factors[8] = {.01, .01, .01, .01, .01, .01, .01, .01};

        double some_period = 100; // WTF is this
        int counter = 0;

        // from optimizaation
        std::string::size_type sz,sz_sub;     // alias of size_t
        cv::Mat drawImg_ = inpImg_.clone();
        int image_center = drawImg_.cols/2;
        int image_height = drawImg_.rows;

        ceres::Problem problem;
        int crop_row_center;
        double gt_period;

        int gt_base_center = image_center + ground_truth.at(0).first;
        double gt_base_period = ground_truth.at(0).second;

        for (uint row_num = 0; row_num < image_height; row_num++) {
            crop_row_center = image_center + ground_truth.at(row_num).first;

            gt_period = ground_truth.at(row_num).second;

            ceres::CostFunction* cost_function = Residual::Create(row_num, crop_row_center, gt_base_center, 0);
            problem.AddResidualBlock(cost_function, NULL, polynomial, perspective_factors, &some_period);

            ceres::CostFunction* cost_function_ = Residual::Create(row_num, crop_row_center + gt_period,
                                                                   gt_base_center + gt_base_period, 1);
            problem.AddResidualBlock(cost_function_, NULL, polynomial, perspective_factors, &some_period);

            ceres::CostFunction* cost_function__ = Residual::Create(row_num, crop_row_center - gt_period,
                                                                    gt_base_center + gt_base_period, -1);
            problem.AddResidualBlock(cost_function__, NULL, polynomial, perspective_factors, &some_period);
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 50;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);


        std::cout << polynomial[0] << " " << polynomial[1] << " " << polynomial[2] << " " << polynomial[3] << " " <<
                  polynomial[4] << " " << some_period << '\n';

        std::cout << perspective_factors[0] << " " << perspective_factors[1] << " " << perspective_factors[2] << " " << perspective_factors[3] << " " <<
                  perspective_factors[4] << " " << perspective_factors[5] << " " << perspective_factors[6] << " " << perspective_factors[7] << '\n';

        double x0 = gt_base_center;

        double x0_right = gt_base_center + gt_base_period;
        double x0_2right = gt_base_center + 2 * gt_base_period;
        double x0_3right = gt_base_center + 3 * gt_base_period;

        double x0_left = gt_base_center - gt_base_period;
        double x0_2left = gt_base_center - 2 * gt_base_period;

        float y0;
        for(unsigned int r = 0; r < inpImg_.rows-1; r++) {
            y0 = (float) eval_poly(polynomial, perspective_factors, r, x0);
            cv::circle(drawImg_, cv::Point2f(y0, r), 2, cv::Scalar(0, 255, 0), 2);

            y0 = (float) (eval_poly(polynomial, perspective_factors, r, x0_right) + some_period);
            cv::circle(drawImg_, cv::Point2f(y0, r), 2, cv::Scalar(0, 255, 0), 2);

            y0 = (float) (eval_poly(polynomial, perspective_factors, r, x0_left) - some_period);
            cv::circle(drawImg_, cv::Point2f(y0, r), 2, cv::Scalar(0, 255, 0), 2);

            y0 = (float) (eval_poly(polynomial, perspective_factors, r, x0_2right) + 2*some_period);
            cv::circle(drawImg_, cv::Point2f(y0, r), 2, cv::Scalar(255, 0, 0), 2);

            y0 = (float) (eval_poly(polynomial, perspective_factors, r, x0_2left) - 2*some_period);
            cv::circle(drawImg_, cv::Point2f(y0, r), 2, cv::Scalar(255, 0, 0), 2);

            y0 = (float) (eval_poly(polynomial, perspective_factors, r, x0_3right) + 3*some_period);
            cv::circle(drawImg_, cv::Point2f(y0, r), 2, cv::Scalar(255, 0, 0), 2);
        }

        cv::imshow("drawImg_", drawImg_);
        //     cv::imwrite("/home/temistocle/Scrivania/ciao.jpg", drawImg_);
        cv::waitKey(0);
    }

    int main_old(int argc, char **argv) {
        // if (argc != 2) {
        //     return -1;
        // }
        // std::map<std::string, double> settings; // setup();
        // settings["a0"] = 1.28;
        // settings["b0"] = 4.48;
        // settings["width"] = 400;
        // settings["height"] = 300;
        // // settings["d_min"] = 8;
        // // settings["n_samples_per_octave"] = 70;
        // // settings["n_octaves"] = 5;

        // cv::Size image_size = cv::Size((uint) settings["width"], (uint) settings["height"]);
        // ImagePreprocessor preprocessor(argv[1], image_size);

        // CropRowDetector row_detector = CropRowDetector();

        // cv::Mat intensityImg = preprocessor.process("path/to/img");

        // row_detector.load(image_size);
        // row_detector.template_matching(intensityImg);
        // std::vector<old_tuple_type> min_energy_results = row_detector.find_best_parameters(row_detector.m_energy_map, row_detector.m_best_energy_by_row);

        // plot_template_matching(intensityImg, min_energy_results);

        // row_detector.teardown();

        // std::cout << "done" << std::endl;
        return 0;
    }
}

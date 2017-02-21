#include <ros/ros.h>

#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "RVLCore.h"
#include "RVLCropRowDetector.h"

#include <CropRowDetector.h>
#include <crop_row_detection.h>
#include <ImagePreprocessor.h>

#include <ceres/ceres.h>

CRVLCropRowDetector CRD;
CRVLVisionSystem VS;
int w, h;

void initDetector(std::string cfg_filename) {
    char ConfigFileName[255];
    strcpy(ConfigFileName, cfg_filename.c_str());

    VS.CreateParamList();

    VS.Init(ConfigFileName);

    w = VS.m_AImage.m_Width;
    h = VS.m_AImage.m_Height;

    CRD.CreateParamList(&(VS.m_Mem0));

    CRD.m_ParamList.LoadParams(ConfigFileName);
    CRD.Init(h);
    // CRD parameters
    double C = 0.0;
    double D = 300.0;
    double f = 300.0;
    double beta = -30.0 * DEG2RAD;
    double z = 1500.0;
    int n = 31;
    double L = 500000.0;
    double s = 100.0;
    double std = 0.1 * s;
    double noiseDensity = 1.0 / (100.0 * 100.0);
}

void image_cb(const sensor_msgs::ImageConstPtr &img_msg) {
    cv::Mat im = cv_bridge::toCvShare(img_msg, "bgr8")->image;

    cv::resize(im, im, cv::Size(w, h));

    IplImage *pInputImage = new IplImage(im);
    IplImage *ExGImage;
    IplImage *pDisplay = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

    ExGImage = cvCreateImage(cvSize(w, h), 8, 1);
    CRD.ExGImage((unsigned char *) pInputImage->imageData, (unsigned char *) ExGImage->imageData, pInputImage->width,
                 pInputImage->height);
//   ExGImage=pInputImage;

    //Apply crop row detection method
    CRD.Apply((unsigned char *) ExGImage->imageData, w, 0);

//   cvCvtColor(pInputImage, pDisplay, CV_GRAY2RGB);
    cvCopy(pInputImage, pDisplay);

    CRD.Display((unsigned char *) (pDisplay->imageData), w);
    cvShowImage("Crop Rows BGR", pDisplay);

    cvWaitKey(10);
}

void
score_xcorr(const crd_cpp::CropRowDetector &row_detector, const std::vector<crd_cpp::old_tuple_type> &min_energy_results) {
    RVLCRD_DPDATA *DP_ = CRD.m_DPData;
    RVLCRD_DPDATA *DP__, *pDP, *pDP_;
    // std::string a;

    double xcorr_err = 0;
    int values = 0;
    for (uint image_row_num = 0; image_row_num < min_energy_results.size(); image_row_num++, DP_ += row_detector.m_row_size) {
        DP__ = DP_;

        for (crd_cpp::period_idx_type period_idx = 0; period_idx < CRD.m_nd; period_idx++, DP__ += CRD.m_nc) {
            crd_cpp::period_type period = row_detector.m_periods[period_idx];
            const crd_cpp::phase_type num_phases = (const crd_cpp::phase_type) floor(period);
            crd_cpp::phase_type first_phase = -num_phases / 2;
            crd_cpp::phase_type last_phase = num_phases + first_phase;

            // original
            int crange = (int) floor(0.5 * period);
            pDP = DP__ + CRD.m_nc / 2 - crange;
            for (crd_cpp::phase_type phase = first_phase; phase < last_phase; phase++, pDP++) {
                const crd_cpp::phase_type real_phase = row_detector.get_real_phase(phase, period);
                auto phase_idx = real_phase - first_phase;

                const crd_cpp::energy_type energy = row_detector.m_energy_map.at(image_row_num).at(period_idx).at(phase_idx);
                auto target_energy = pDP->D;
                auto error = target_energy - energy;
                xcorr_err += std::abs(error);
                values++;
            }
        }
    }
    std::cout << "xcorr err:" << xcorr_err / values << std::endl;
}

void
score_optimization(const crd_cpp::CropRowDetector &row_detector, std::vector<crd_cpp::old_tuple_type> &min_energy_results) {
    double total_err = 0;
    for (size_t image_row_num = 0; image_row_num < min_energy_results.size(); image_row_num++) {
        auto x = min_energy_results.at(image_row_num);
        int phase = x.first;
        double period = x.second;
        int period_idx = -1;

        while(period != row_detector.m_periods[++period_idx]);

		int target_phase = (int) CRD.m_c[image_row_num];
		int target_period_idx = (int) CRD.m_id[image_row_num];

        int phase_error = target_phase - phase;
        int period_idx_error = target_period_idx - period_idx;
        // auto distance = std::sqrt(std::pow(phase_error, 2) + std::pow(period_idx_error, 2));
        auto distance = std::abs(phase_error) + std::abs(period_idx_error);
        total_err += distance;

        // std::cout << "row: " << image_row_num
        //           << " phase err: " << phase_error
        //           << " period err: " << period_idx_error
        //           << " err: " << distance
        //           << std::endl;
	}
    std::cout << "avg err:" << total_err / min_energy_results.size() << std::endl;
}

void setup_crds(char **argv, int argc, crd_cpp::CropRowDetector &row_detector) {

    //---------- CROP ROW DETECTION ORIGINAL SETUP ----------
    ros::init(argc, argv, "crop_row_detector");
    ros::NodeHandle nh;
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 1, image_cb);

    std::string cfg_filename = "/home/noflip/catkin_ws/src/crop_row_detection/crop_row_detection_original/cfg/params.cfg";
    cfg_filename = nh.param("cfg_filename", cfg_filename);

    std::cout << cfg_filename << std::endl;

    if (cfg_filename.length() < 1) {
        std::cout << "Usage: _cfg_filename:=/..." << std::endl;
        ROS_INFO("Config file does not exist!");
        exit(1);
    }
    std::cout << "initingDetector ..";
    initDetector(cfg_filename);
    std::cout << "done" << std::endl;

    //---------- CROP ROW DETECTION CPP SETUP ----------
    cv::Size image_size = cv::Size(400, 300);

    std::cout << "loading" << std::endl;
    row_detector.load(image_size);
    std::cout << "loaded" << std::endl;
}

int main(int argc, char **argv) {
    const bool USE_CPP = false;

    crd_cpp::CropRowDetector row_detector = crd_cpp::CropRowDetector();
    clock_t start;
    std::vector<crd_cpp::old_tuple_type> min_energy_results;

    setup_crds(argv, argc, row_detector);

    crd_cpp::ImagePreprocessor preprocessor = crd_cpp::ImagePreprocessor(cv::Size(400, 300));
    std::cout << "preprocessor initd";

    std::string video_file = "/home/noflip/catkin_ws/src/crop_row_detection/score_results/Sugarbeets_Field.mp4";
    cv::VideoCapture capture(video_file);
    cv::Mat video_image;
    while(capture.isOpened()){
        capture >> video_image;
        // cpp image processing
        cv::Mat intensityImage = preprocessor.process(video_image);
        if(USE_CPP){
            // templte matching for cpp
            start = std::clock();
            row_detector.template_matching(intensityImage);
            std::cout << "template_matching time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

            // optimization for cpp
            start = std::clock();
            std::cout << "optimization:" << std::endl;
            min_energy_results = row_detector.find_best_parameters(row_detector.m_energy_map, row_detector.m_best_energy_by_row);
            std::cout << "best_param time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

            // show results for CPP
            std::cout << "plotting" << std::endl;
            // dump cpp results
            crd_cpp::dump_template_matching(min_energy_results, row_detector.m_image_width, "cpp");

            if(argc > 1)
                crd_cpp::plot_template_matching(intensityImage, min_energy_results);
        }

        // original imge preprocessing
        std::cout << "resizing.." << std::endl;
        cv::resize(video_image, video_image, cv::Size(w, h));
        IplImage *pInputImage = new IplImage(video_image);
        IplImage *ExGImage;
        IplImage *pDisplay = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
        std::cout << "creating ExG.." << std::endl;
        ExGImage = cvCreateImage(cvSize(w, h), 8, 1);


        // preprocess original
        CRD.ExGImage((unsigned char *) pInputImage->imageData, (unsigned char *) ExGImage->imageData, pInputImage->width, pInputImage->height);
        cvShowImage("Crop Rows BGR", ExGImage);
        cvSaveImage("ExG.png", ExGImage);

        //Apply crop row detection method
        std::cout << "Start Apply" << std::endl;
        start = std::clock();
        CRD.Apply((unsigned char *) ExGImage->imageData, w, 0);
        std::cout << "apply time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

        // dump original results
        std::vector<crd_cpp::old_tuple_type> crd_original(h);
        for (uint image_row_num = 0; image_row_num < h; image_row_num++)
            crd_original.at(image_row_num) = crd_cpp::old_tuple_type(CRD.m_c[image_row_num], CRD.m_d[image_row_num]);
        crd_cpp::dump_template_matching(crd_original, w, "original");
        std::cout << ">fitting initial guess" << std::endl;
        crd_cpp::Polyfit polyfit(video_image, intensityImage, crd_original);

        std::cout << ">adding noise" << std::endl;
        // polyfit.add_noise();

        std::cout << ">refitting" << std::endl;
        polyfit.fit(video_image);
    }


    // show original results
    //   cvCvtColor(pInputImage, pDisplay, CV_GRAY2RGB);
    // cvCopy(pInputImage, pDisplay);
    // CRD.Display((unsigned char *) (pDisplay->imageData), w);
    // cvShowImage("Crop Rows BGR", pDisplay);
    // cvWaitKey(0);

    if(USE_CPP){
        // compare scores
        std::cout << "checking template matching" << std::endl;
        score_xcorr(row_detector, min_energy_results);
        score_optimization(row_detector, min_energy_results);

        // teardown cpp
        std::cout << "teardown" << std::endl;
        row_detector.teardown();
    }

    // while (ros::ok())
    //     ros::spinOnce();
    // ros::spin();

    return 0;
}

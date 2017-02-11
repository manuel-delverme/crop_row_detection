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

using namespace std;
using namespace cv;

CRVLCropRowDetector CRD;
CRVLVisionSystem VS;
int w, h;

void initDetector(string cfg_filename) {
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

    resize(im, im, Size(w, h));

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "crop_row_detector");

    ros::NodeHandle nh;

    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("/camera/image_raw", 1, image_cb);

    string cfg_filename = "/home/noflip/catkin_ws/src/crop_row_detection/crop_row_detection_original/cfg/params.cfg";

    cfg_filename = nh.param("cfg_filename", cfg_filename);

    cout << cfg_filename << endl;

    if (cfg_filename.length() < 1) {
        cout << "Usage: _cfg_filename:=/..." << endl;
        ROS_INFO("Config file does not exist!");
        return 0;
    }
    cout << "initingDetector .." << std::endl;
    initDetector(cfg_filename);

    string image_path = "/home/noflip/catkin_ws/src/crop_row_detection/crop_row_detection_cpp/Images/download.jpg";
    string image_path_cpp = "/home/noflip/catkin_ws/src/crop_row_detection/crop_row_detection_cpp/Images/";
    cout << "loading image..";
    cv::Mat im = imread(image_path);
    cout << "resizing.." << endl;
    resize(im, im, Size(w, h));

    IplImage *pInputImage = new IplImage(im);
    IplImage *ExGImage;
    IplImage *pDisplay = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);

    cout << "creating ExG.." << endl;
    ExGImage = cvCreateImage(cvSize(w, h), 8, 1);

    //---------- CROP ROW DETECTION CPP SETUP ----------
    cout << "CPP setup" << endl;
    std::map<std::string, double> settings; // setup();
    settings["a0"] = 1.28;
    settings["b0"] = 4.48;
    settings["width"] = 400;
    settings["height"] = 300;
    cv::Size image_size = cv::Size((uint) settings["width"], (uint) settings["height"]);
    crd_cpp::ImagePreprocessor preprocessor(image_path_cpp, image_size);
    crd_cpp::CropRowDetector row_detector = crd_cpp::CropRowDetector();

    std::vector<std::vector<std::vector<crd_cpp::energy_type>>> energy_map;
    std::vector<cv::Mat> images = preprocessor.process();
    cv::Mat &pIntensityImg = images.at(0);

    std::cout << "loading" << std::endl;
    row_detector.load(pIntensityImg);

    std::clock_t start;


    std::cout << "loaded" << std::endl;

    start = std::clock();
    row_detector.graph_search();
    std::cout << "graph time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
    exit(1);

    start = std::clock();
    row_detector.template_matching();
    std::cout << "template_matching time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    std::cout << "optimization:" << std::endl;
    start = std::clock();
    auto min_energy_results = row_detector.find_best_parameters(row_detector.m_energy_map, row_detector.m_best_energy_by_row);
    std::cout << "best_param time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    CRD.ExGImage((unsigned char *) pInputImage->imageData, (unsigned char *) ExGImage->imageData, pInputImage->width, pInputImage->height);

    std::cout << "teardown" << std::endl;
    row_detector.teardown();

    std::cout << "plotting" << std::endl;

    if(argc > 1){
        crd_cpp::plot_template_matching(pIntensityImg, min_energy_results);
        cvShowImage("Crop Rows BGR", ExGImage);
        cvSaveImage("ExG.png", ExGImage);
    }

    //Apply crop row detection method
    std::cout << "START" << std::endl;
    start = std::clock();
    CRD.Apply((unsigned char *) ExGImage->imageData, w, 0);
    std::cout << "apply time: " << (std::clock() - start) / (double) (CLOCKS_PER_SEC / 1000) << " ms" << std::endl;

    cvCopy(pInputImage, pDisplay);

    CRD.Display((unsigned char *) (pDisplay->imageData), w);
    if(argc > 1)
        cvShowImage("Crop Rows BGR", pDisplay);
        cvWaitKey(0);



    // while (ros::ok())
    //     ros::spinOnce();
    // ros::spin();

    return 0;
}

// ros
// #include <ros/ros.h>
// #include <ros/subscriber.h>
// #include <sensor_msgs/Image.h>

// cv3
// #include <opencv3/highgui/highgui.hpp>
// #include <opencv3/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// #include <eigen/core>
#include <iostream>

class ImagePreprocessor{
    public:
        ImagePreprocessor (std::string images_path, cv::Size target_size);
        std::vector<cv::Mat> process();
    private:
        cv::Size m_size;
        std::string m_images_folder;
        cv::Mat convertToExG(cv::Mat&);
};
ImagePreprocessor::ImagePreprocessor(std::string images_path, cv::Size target_size){
    std::cout << "initd ImagePreprocessor " << std::endl;
    m_size = target_size;
    m_images_folder = images_path;
}
cv::Mat ImagePreprocessor::convertToExG(cv::Mat &image){
    cv::Mat intensity = cv::Mat::zeros(image.size(), CV_8UC1);
    cv::Mat bgr[3];   //destination array
    cv::split(image, bgr);
    intensity = 2 * bgr[1] - bgr[2] - bgr[0];
    return intensity;
};
std::vector<cv::Mat> ImagePreprocessor::process(){
    std::vector<cv::Mat> images;
    cv::String path(m_images_folder + "*"); //select only jpg
    std::vector<cv::String> file_names;
    cv::glob(path, file_names, true); // recurse

    std::cout << "resizing images to " << m_size  << std::endl;
    for (std::string file_path : file_names)
    {
        cv::Mat image = cv::imread(file_path, CV_LOAD_IMAGE_COLOR);
        // if (im.empty()) continue; //only proceed if
        cv::Mat small_image;
        cv::resize(image, small_image, m_size); // settings["image_size"]);
        cv::Mat intensity_map = ImagePreprocessor::convertToExG(image);
        images.push_back(intensity_map);
    }
    return images;
}

class CropRowDetector{
        // int width;
    public:
        CropRowDetector (int);
        int area () {return 1;}
        // CropRowDetector(std::map<std::string, double>);
        void setup();
        cv::Mat detect(cv::Mat& intensity_map, cv::Mat& temp);
};

CropRowDetector::CropRowDetector (int a) {
    std::cout << "initd" << a << std::endl;
}

cv::Mat CropRowDetector::detect(cv::Mat& intensity, cv::Mat& templ){
    // Mat templ; // from settings
    cv::Mat result;
    int result_rows = intensity.rows - templ.rows;
    int result_cols = intensity.cols - templ.cols;
    result.create(result_rows, result_cols, CV_8UC1);
    cv::matchTemplate(intensity, templ, result, CV_TM_CCORR);
    // normalize?
    return result;
}

void CropRowDetector::setup(){
    // ros::init(argc, argv, "crd_node");
    // ros::NodeHandle node_handle;
    // ros::Subscriber img_sub = node_handle.subscribe<sensor_msgs::Image>
    //     ("/camera/image_raw", 1, image_callback);

    // string cfg_filename;
    // cfg_filename = node_handle.param("crop_row_detector/cfg_filename",
    //         cfg_filename);

    // if(cfg_filename.length()<1)
    // {
    //   cerr << cfg_filename << " not found, using default"  << endl;
    //   cfg_filename = "config/config.cfg";
    //   ROS_INFO("Config file does not exist!");
    // }
    // config = load_config(cfg_filename);
    std::cout << "setted up" << std::endl;
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
void display_img(cv::Mat image){
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Display Image", image);
    cv::waitKey(0);
}

int main(int argc, char** argv){
    std::map<std::string, double> settings; // setup();

    /*
     *  preprocess img
     *  - downsample
     *  - exg
     */
    if(argc < 2){
        return -1;
    }
    std::cout << "initing preprocess" << std::endl;
    ImagePreprocessor preprocessor (argv[1], cv::Size(100,100));
    std::cout << "process" << std::endl;
    std::vector<cv::Mat> data = preprocessor.process();
    //TODO test results against paper-generated
    CropRowDetector row_detector (42);
    // za from 0 to a/2
    // 0 from a/2 to (0.5 - b/2)
    // -za from (0.5 - b/2) to (0.5 + b/2)
    // 0 from (0.5 + b/2) to 1-a/2
    // +za from 1-a/2 to 1
    cv::Mat templ = cv::Mat::zeros(1, data[0].cols, CV_8UC1);

    int a = 10;
    int b = 20;
    int za = 1;
    int zb = -1;
    for(int y=0; y<50; y+=1){
        if(y <= a/2 || y >= 1-a/2)
        {
            templ.at<uchar>(1, y) = za;
        }
        else if( (1-b)/2 <= y && y <= (1+b)/2 ){
            templ.at<uchar>(1, y) = -zb;
        }
    }

    for (cv::Mat& pIntensityImg : data) {
        std::cout << "imshow" << std::endl;
        display_img(pIntensityImg);
        row_detector.detect(pIntensityImg, templ);
    }

    // ros::spin();
    //  cv::ExGImage(small_image, intensity_map);
    std::cout << "done" << std::endl;
    return 0;
}

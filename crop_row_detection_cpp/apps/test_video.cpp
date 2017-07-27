#include <opencv2/opencv.hpp>

#include <ctime>

#include <CropRowDetector.h>
#include <crop_row_detection.h>
#include <ImagePreprocessor.h>

#include <ceres/ceres.h>

int main ( int argc, char **argv )
{
    // crd_cpp::CropRowDetector row_detector;
    clock_t start;

    cv::Size image_size = cv::Size ( 400, 300 );

    std::cout << "loading" << std::endl;
    // row_detector.pre_alloc(image_size);
    std::cout << "loaded" << std::endl;

    crd_cpp::ImagePreprocessor preprocessor( image_size );
    std::cout << "preprocessor initd";

    std::string video_file = argv[1];
    const int initial_fit_max_iter = atoi(argv[2]);
    const int initial_fit_max_useless_iter = atoi(argv[3]);
    const int max_iter = atoi(argv[4]);
    const int max_useless_iter = atoi(argv[5]);

    cv::VideoCapture capture ( video_file );
    cv::Mat video_image;
    cv::Mat intensityImage;
    std::vector<crd_cpp::old_tuple_type> min_energy_results;

    capture >> video_image;
    cv::resize(video_image, video_image, image_size);
    std::cout << "fitting initial guess" << std::endl;
    // resize image?
    intensityImage = preprocessor.process(video_image);
    // cv::imshow("a", intensityImage);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    crd_cpp::Polyfit polyfit (video_image, intensityImage, initial_fit_max_iter, initial_fit_max_useless_iter);

    // cv::Mat img = cv::imread(image_file_name);
    // cv::resize(img, img, image_size);
    // cv::Mat intensityImage = preprocessor.process(img);

    while(capture.isOpened()) {
        capture >> video_image;
        intensityImage = preprocessor.process (video_image);
        polyfit.fit(video_image, max_iter, max_useless_iter);
        break;
    }
    // teardown cp
    std::cout << "teardown" << std::endl;
    // row_detector.teardown();
    return 0;
}

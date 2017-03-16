#include <opencv2/opencv.hpp>

#include <ctime>

#include <CropRowDetector.h>
#include <crop_row_detection.h>
#include <ImagePreprocessor.h>

#include <ceres/ceres.h>

int main ( int argc, char **argv )
{
    crd_cpp::CropRowDetector row_detector;
    clock_t start;

    cv::Size image_size = cv::Size ( 400, 300 );

    std::cout << "loading" << std::endl;
    row_detector.pre_alloc(image_size);
    std::cout << "loaded" << std::endl;

    crd_cpp::ImagePreprocessor preprocessor( image_size );
    std::cout << "preprocessor initd";

    std::string video_file = argv[1];
    cv::VideoCapture capture ( video_file );
    cv::Mat video_image;
    cv::Mat intensityImage;
    std::vector<crd_cpp::old_tuple_type> min_energy_results;

    capture >> video_image;
    std::cout << "fitting initial guess" << std::endl;
    intensityImage = preprocessor.process (video_image );
    crd_cpp::Polyfit polyfit ( video_image, intensityImage, min_energy_results);

    while ( capture.isOpened() )
    {
        capture >> video_image;
        // cpp image processing
        intensityImage = preprocessor.process (video_image );

        // templte matching for cpp
        start = std::clock();
        row_detector.template_matching ( intensityImage );
        std::cout << "template_matching time: " << ( std::clock() - start ) / ( double ) ( CLOCKS_PER_SEC / 1000 ) << " ms" << std::endl;

        // optimization for cpp
        start = std::clock();
        std::cout << "optimization:" << std::endl;
        min_energy_results = row_detector.find_best_parameters (row_detector.m_energy_map, row_detector.m_best_energy_by_row );
        std::cout << "best_param time: " << ( std::clock() - start ) / ( double ) ( CLOCKS_PER_SEC / 1000 ) << " ms" << std::endl;

        if ( argc > 2 )
            // show results for CPP
            std::cout << "plotting" << std::endl;
            crd_cpp::plot_template_matching ( intensityImage, min_energy_results );

        std::cout << ">refitting" << std::endl;
        polyfit.fit ( video_image );
    }

    // teardown cpp
    std::cout << "teardown" << std::endl;
    row_detector.teardown();

    return 0;
}

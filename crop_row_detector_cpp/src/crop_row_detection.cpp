#include <iostream>
#include <map>

// CropRowDetector(std::map<std::string, double> settings){
CropRowDetector::CropRowDetector(int param){
    std::cout << "loaded" << param << std::endl;
};

void CropRowDetector::setup(){
    /*
    ros::init(argc, argv, "crd_node");
    ros::NodeHandle node_handle;
    ros::Subscriber img_sub = node_handle.subscribe<sensor_msgs::Image>
        ("/camera/image_raw", 1, image_callback);

    string cfg_filename;
    cfg_filename = node_handle.param("crop_row_detector/cfg_filename",
            cfg_filename);

    if(cfg_filename.length()<1)
    {
      cerr << cfg_filename << " not found, using default"  << endl;
      cfg_filename = "config/config.cfg";
      // ROS_INFO("Config file does not exist!");
    }
    config = load_config(cfg_filename);
    */
}

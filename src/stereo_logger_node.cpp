#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <signal.h>
#include <cstdlib>
#include <unistd.h>
    
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "stereo_logger_node.h"

StereoLoggerNode::StereoLoggerNode(const std::string node_name, rclcpp::NodeOptions options)
: Node(node_name, options)
{
    std::string camera_left_topic = this->declare_parameter<std::string>("subscribers.camera_left", "/camera_left/raw_frame");
    std::string camera_right_topic = this->declare_parameter<std::string>("subscribers.camera_right", "/camera_right/raw_frame");

    frame_left_sub = this->create_subscription<Image>(camera_left_topic, 1, 
        std::bind(&StereoLoggerNode::callback_frame_left, this, std::placeholders::_1));

    frame_right_sub = this->create_subscription<Image>(camera_right_topic, 1, 
        std::bind(&StereoLoggerNode::callback_frame_right, this, std::placeholders::_1));

    storage_folder = this->declare_parameter<std::string>("storage_folder", "auto");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("stereo_logger");

    if (storage_folder.compare("auto") == 0){
        storage_folder = package_share_directory + "/saved_images/";
    }

    boost_fs::path storage_folder_path = storage_folder;

    if(boost::filesystem::create_directory(storage_folder_path))
    {
        std::cerr<< "Directory Created: "<< storage_folder_path << std::endl;
    }
    std::string storage_folder_path_str = boost::filesystem::canonical(storage_folder_path).string();

    std::string cmd_to_empty_folder = "exec rm -rf " + storage_folder_path_str + "/*";
    system(cmd_to_empty_folder.c_str());

    storage_folder_path_left = storage_folder / boost_fs::path("left");
    storage_folder_path_right = storage_folder / boost_fs::path("right");

    if(boost::filesystem::create_directory(storage_folder_path_left))
    {
        std::cerr<< "Directory Created: "<< storage_folder_path_left << std::endl;
    }
    if(boost::filesystem::create_directory(storage_folder_path_right))
    {
        std::cerr<< "Directory Created: "<< storage_folder_path_right << std::endl;
    }

    std::cerr << "storage_folder_path_left: " << storage_folder_path_left << std::endl;
    std::cerr << "storage_folder_path_right: " << storage_folder_path_right << std::endl;

    float hz_store_image = this->declare_parameter<float>("hz_store_image", 0.5);

    padding = this->declare_parameter<int>("padding_image_name", 4);

    store_image_timer = this->create_wall_timer(1000ms / hz_store_image, std::bind(&StereoLoggerNode::store_images, this));
    

}

void StereoLoggerNode::callback_frame_left(const Image::SharedPtr msg) {
    
    cv_bridge::CvImagePtr cv_ptr;
        
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    current_frame_left = cv_ptr->image;
}

void StereoLoggerNode::callback_frame_right(const Image::SharedPtr msg) {
    
    cv_bridge::CvImagePtr cv_ptr;
        
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    current_frame_right = cv_ptr->image;
    
}
     

void StereoLoggerNode::store_images(){

    if(current_frame_left.empty() or current_frame_right.empty())
    {
        std::cerr << "Waiting for both frame acquisition to start..." << std::endl;
        return;
    }
    else
    {
        cv::Mat img_left = current_frame_left;
        cv::Mat img_right = current_frame_right;

        std::stringstream ss;
        ss << std::setw(padding) << std::setfill('0') << incremental_index;
        std::string count_to_string = ss.str();

        std::string img_left_path = boost::filesystem::canonical(storage_folder_path_left).string() + "/" + count_to_string + ".png";;

        std::string img_right_path = boost::filesystem::canonical(storage_folder_path_right).string() + "/" + count_to_string + ".png";;
            
        cv::imwrite(img_left_path, img_left);
        cv::imwrite(img_right_path, img_right);

        std::cerr << "Acquired pair #" << (incremental_index+1) << std::endl;

        incremental_index++;
    }
    
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();

    rclcpp::NodeOptions node_options;

    node_options.use_intra_process_comms(true);
    auto node = std::make_shared<StereoLoggerNode>("stereo_logger", node_options);

    executor->add_node(node);
    executor->spin(); 

    rclcpp::shutdown();

    return 0;
}

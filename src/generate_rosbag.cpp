#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tum_rosbag");

    ros::NodeHandle nh("~");

    std::string dataset_folder, sequence_name, rosbag_path;
    nh.getParam("dataset_folder", dataset_folder);
    nh.getParam("sequence_name", sequence_name);
    nh.getParam("rosbag_path", rosbag_path);

    std::cout << "Reading sequence " << '\n';

    std::cout << dataset_folder << std::endl;
    std::cout << sequence_name << std::endl;
    std::cout << rosbag_path << std::endl;

    std::cout << "any results?" << '\n';

    std::string image_directory = dataset_folder+sequence_name+"/rgb/";
    std::string rosbagfile_path = rosbag_path+sequence_name+".bag";

    std::cout << rosbagfile_path << std::endl;

    cv::Mat rgb_image;
    sensor_msgs::ImagePtr rgb_msg;

    rosbag::Bag bag;
    bag.open(rosbagfile_path, rosbag::bagmode::Write);

    boost::filesystem::path dir(image_directory);
    std::vector<std::string> image_files;
    for (boost::filesystem::directory_entry& entry : boost::filesystem::directory_iterator(dir)) {
        if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".png") {
            image_files.push_back(entry.path().string());
        }
    }

    int total_images = image_files.size();
    int images_processed = 0;

    for (const std::string& image_path : image_files) {
        std::string filename = boost::filesystem::path(image_path).stem().string();
        double timestamp = std::stod(filename);
        rgb_image = cv::imread(image_path, cv::IMREAD_COLOR);
        rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb_image).toImageMsg();
        rgb_msg->header.stamp = ros::Time(timestamp);
        rgb_msg->header.frame_id = "/camera_color_optical_frame";
        bag.write("/camera/color/image_raw", ros::Time(timestamp), *rgb_msg);
        std::cout << "Progress: " << (images_processed * 100 / total_images) << "%" << std::endl;

        images_processed++;
    }



    /*
    RGB:
    seq ??
    frame id        camera_color_optical_frame
    encoding        "rgb8"
    is_bigendian: 0


    /camera/depth/color/points   2603 msgs    : sensor_msgs/PointCloud2
        frame id        camera_depth_optical_frame
        height: 1       width: xxxx

    /tf_static                      1 msg     : tf2_msgs/TFMessage
             

    */ 
    

    
    bag.close();
    return 0;
}
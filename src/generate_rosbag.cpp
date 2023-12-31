#include <ros/ros.h>
#include <rosbag/bag.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>

#include <iostream>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tum_rosbag");

    ros::NodeHandle nh("~");

    std::string dataset_folder, sequence_name, rosbag_path;
    nh.getParam("dataset_folder", dataset_folder);
    nh.getParam("sequence_name", sequence_name);
    nh.getParam("rosbag_path", rosbag_path);

    std::cout << dataset_folder << std::endl;
    std::cout << sequence_name << std::endl;
    std::cout << rosbag_path << std::endl;

    std::string image_directory = dataset_folder+sequence_name+"/rgb/";
    std::string depth_directory = dataset_folder+sequence_name+"/depth/";
    std::string rosbagfile_path = rosbag_path+sequence_name+".bag";

    std::cout << rosbagfile_path << std::endl;

    
    

    rosbag::Bag bag;
    bag.open(rosbagfile_path, rosbag::bagmode::Write);

    // write tf_static
    tf2_msgs::TFMessage tf_msg;
    geometry_msgs::TransformStamped tf1, tf2, tf3, tf4;
    tf1.header.stamp = ros::Time::now();
    tf1.header.frame_id = "camera_link";
    tf1.child_frame_id = "camera_depth_frame";
    tf1.transform.translation.x = 0.0;
    tf1.transform.translation.y = 0.0;
    tf1.transform.translation.z = 0.0;
    tf1.transform.rotation.x = 0.0;
    tf1.transform.rotation.y = 0.0;
    tf1.transform.rotation.z = 0.0;
    tf1.transform.rotation.w = 1.0;
    tf2.header.stamp = ros::Time::now();
    tf2.header.frame_id = "camera_depth_frame";
    tf2.child_frame_id = "camera_depth_optical_frame";
    tf2.transform.translation.x = 0.0;
    tf2.transform.translation.y = 0.0;
    tf2.transform.translation.z = 0.0;
    tf2.transform.rotation.x = -0.5;
    tf2.transform.rotation.y = 0.5;
    tf2.transform.rotation.z = -0.5;
    tf2.transform.rotation.w = 0.5;
    tf3.header.stamp = ros::Time::now();
    tf3.header.frame_id = "camera_link";
    tf3.child_frame_id = "camera_color_frame";
    tf3.transform.translation.x = 0.00643118284643;
    tf3.transform.translation.y = 0.000696854724083;
    tf3.transform.translation.z = 0.0142132211477;
    tf3.transform.rotation.x = 0.00256140390411;
    tf3.transform.rotation.y = -0.0124211823568;
    tf3.transform.rotation.z = -0.00104970496614;
    tf3.transform.rotation.w = 0.999918997288;
    tf4.header.stamp = ros::Time::now();
    tf4.header.frame_id = "camera_color_frame";
    tf4.child_frame_id = "camera_color_optical_frame";
    tf4.transform.translation.x = 0.0;
    tf4.transform.translation.y = -0.0;
    tf4.transform.translation.z = -0.0;
    tf4.transform.rotation.x = -0.5;
    tf4.transform.rotation.y = 0.5;
    tf4.transform.rotation.z = -0.5;
    tf4.transform.rotation.w = 0.5;
    tf_msg.transforms.push_back(tf1);
    tf_msg.transforms.push_back(tf2);
    tf_msg.transforms.push_back(tf3);
    tf_msg.transforms.push_back(tf4);
    bag.write("tf_static", ros::Time::now(), tf_msg);


    // write rgb image
    boost::filesystem::path image_dir(image_directory);
    std::vector<std::string> image_files;
    for (boost::filesystem::directory_entry& entry : boost::filesystem::directory_iterator(image_dir)) {
        if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".png") {
            image_files.push_back(entry.path().string());
        }
    }
    int total_images = image_files.size();
    int images_processed = 0;
    cv::Mat rgb_image;
    sensor_msgs::ImagePtr rgb_msg;
    for (const std::string& image_path : image_files) {
        std::string image_filename = boost::filesystem::path(image_path).stem().string();
        double timestamp = std::stod(image_filename);
        rgb_image = cv::imread(image_path, cv::IMREAD_COLOR);
        rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
        rgb_msg->header.stamp = ros::Time(timestamp);
        rgb_msg->header.frame_id = "/camera_color_optical_frame";
        bag.write("/camera/color/image_raw", ros::Time(timestamp), *rgb_msg);
        std::cout << "Progress: " << images_processed << "/" << total_images << std::endl;
        ros::Duration(0.01).sleep();
        images_processed++;
    }

    // write pointcloud
    boost::filesystem::path depth_dir(depth_directory);
    std::vector<std::string> depth_files;
    for (boost::filesystem::directory_entry& entry : boost::filesystem::directory_iterator(depth_dir)) {
        if (boost::filesystem::is_regular_file(entry) && entry.path().extension() == ".png") {
            depth_files.push_back(entry.path().string());
        }
    }
    int total_depth = depth_files.size();
    int depth_processed = 0;
    cv::Mat depth_image;
    sensor_msgs::PointCloud2 cloud_msg;
    //freiburg3
    float fx = 535.4;
    float fy = 539.2;
    float cx = 320.1;
    float cy = 247.6;
    float depth_scale = 5000.0; 
    for (const std::string& depth_path : depth_files) {
        std::string depth_filename = boost::filesystem::path(depth_path).stem().string();
        double timestamp = std::stod(depth_filename);
        depth_image = cv::imread(depth_path, cv::IMREAD_UNCHANGED);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int v = 0; v < depth_image.rows; v++) {
            for (int u = 0; u < depth_image.cols; u++) {
                float Z = depth_image.at<uint16_t>(v, u);
               
                if (Z > 0) {
                    Z = Z /depth_scale;
                    pcl::PointXYZRGB point;
                    point.x = (u - cx) * Z / fx;
                    point.y = (v - cy) * Z / fy;
                    point.z = Z;
                    
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                    cloud->points.push_back(point);
                }
            }
        }
        
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time(timestamp);
        cloud_msg.header.frame_id = "/camera_depth_optical_frame";
        bag.write("/camera/depth/color/points", ros::Time(timestamp), cloud_msg);
        std::cout << "Progress: " << depth_processed << "/" << total_depth << std::endl;
        ros::Duration(0.01).sleep();
        depth_processed++;
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
            header: 
            seq: 0
            stamp: 
                secs: 1611576912
                nsecs:  65503599
            frame_id: "camera_color_frame"
            child_frame_id: "camera_color_optical_frame"
            transform: 
            translation: 
                x: 0.0
                y: -0.0
                z: -0.0
            rotation: 
                x: -0.5
                y: 0.5
                z: -0.5
                w: 0.5    

    */ 
    

    
    bag.close();
    return 0;
}
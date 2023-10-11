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
    std::string associations_file = dataset_folder + sequence_name + "/associations.txt";

    std::ifstream associations_stream(associations_file);
    if (!associations_stream.is_open()) {
        ROS_ERROR("Failed to open associations.txt");
        return -1;
    }

    

    
    

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


    float fx = 535.4;
    float fy = 539.2;
    float cx = 320.1;
    float cy = 247.6;
    float depth_scale = 5000.0; 

    int processed = 0;
    std::string line;
    while (std::getline(associations_stream, line)) {
        std::istringstream iss(line);
        double rgb_timestamp, depth_timestamp;
        std::string rgb_file, depth_file;
        if (!(iss >> rgb_timestamp >> rgb_file >> depth_timestamp >> depth_file)) {
            ROS_WARN("Error parsing associations.txt line: %s", line.c_str());
            continue;
        }
        // Process RGB image
        std::string rgb_image_path = dataset_folder + sequence_name + "/" + rgb_file;
        cv::Mat rgb_image = cv::imread(rgb_image_path, cv::IMREAD_COLOR);
        sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
        rgb_msg->header.stamp = ros::Time(rgb_timestamp);
        rgb_msg->header.frame_id = "/camera_color_optical_frame";
        bag.write("/camera/color/image_raw", ros::Time(rgb_timestamp), *rgb_msg);

        // Process depth image
        std::string depth_image_path = dataset_folder + sequence_name + "/" + depth_file;
        cv::Mat depth_image = cv::imread(depth_image_path, cv::IMREAD_UNCHANGED);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        sensor_msgs::PointCloud2 cloud_msg;
        for (int v = 0; v < depth_image.rows; v++) {
            for (int u = 0; u < depth_image.cols; u++) {
                float Z = depth_image.at<uint16_t>(v, u);
                if (Z > 0) {
                    Z = Z /depth_scale;
                    pcl::PointXYZRGB point;
                    point.x = (u - cx) * Z / fx;
                    point.y = (v - cy) * Z / fy;
                    point.z = Z;
                    cv::Vec3b rgb_pixel = rgb_image.at<cv::Vec3b>(v, u);
                    point.r = rgb_pixel[2];
                    point.g = rgb_pixel[1];
                    point.b = rgb_pixel[0];
                    cloud->points.push_back(point);
                }
            }
        }
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = ros::Time(depth_timestamp);
        cloud_msg.header.frame_id = "/camera_depth_optical_frame";
        bag.write("/camera/depth/color/points", ros::Time(depth_timestamp), cloud_msg);

        processed++;

        std::cout << "Progress: " << processed << std::endl;
        ros::Duration(0.01).sleep();
    }

    
    bag.close();
    return 0;
}
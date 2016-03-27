#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "turntable/saveImages.h"

class save_images{
	ros::NodeHandle nh;
	ros::Subscriber kinect_pc_sub;
	ros::ServiceServer service;
  image_transport::ImageTransport it_;
  image_transport::Subscriber kinect_img_sub;
  image_transport::Subscriber mask_sub;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color_pc;
	cv_bridge::CvImagePtr kinect_color_raw;
	cv_bridge::CvImagePtr kinect_color_mask;

	int new_pc;
	int new_raw;
	int new_mask;
  std::map<std::string, std::string> num_2_item_dict;

  public:
  	save_images(): it_(nh){
  		kinect_pc_sub = nh.subscribe("/kinect2/hd/points", 1, &save_images::kinect_pc_cb, this);
  		mask_sub = it_.subscribe("/image_converter/output_video", 1, &save_images::mask_cb, this);
  		kinect_img_sub = it_.subscribe("/kinect2/hd/image_color", 1, &save_images::kinect_img_cb, this);

  		service = nh.advertiseService("save_images", &save_images::save_data_cb, this);

  		//kinect_color_pc = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

      ros::param::get("num_2_item_dict",num_2_item_dict);

      std::cout << "Constructor completed\n";

  	}

  	void kinect_pc_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  		std::cout << "new pc message\n";
  		pcl::fromROSMsg(*cloud_msg, *kinect_color_pc);
  		new_pc = 1;
      std::cout << "new PC!\n";
  	}
  	void kinect_img_cb(const sensor_msgs::ImageConstPtr& msg){
      kinect_color_raw = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  		new_raw = 1;
      std::cout<< "new IMAGE!\n";


  	}
  	void mask_cb(const sensor_msgs::ImageConstPtr& msg){
  		kinect_color_mask = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  		new_mask = 1;
      std::cout << "new MASK!\n";	
	}

	bool save_data_cb(turntable::saveImages::Request &req,  turntable::saveImages::Response &res){
		/*uint16 item_number
		uint16 angle
		uint16 count
		---
		uint64 status
		
		std::cout << "Save Data request recieved: \n"<< 
					 "item_number: "<< req.item_number << 
					 "angle: "<< req.angle <<
					 "count: "<< req.count << std::endl;
           */
    std::cout << "inside save_Data!!!!!!!!!!!\n";

		res.status = 1;
		return true;
	}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_images_server");
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    
    save_images si;
    ROS_INFO("save images server online, awaiting commands...");
    ros::waitForShutdown();
    return 0;
}
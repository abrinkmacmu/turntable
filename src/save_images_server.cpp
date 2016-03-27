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

#include <boost/filesystem.hpp>

#include <ctime>

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
  int waiting_for_new;
  std::string dir_path;
  std::map<std::string, std::string> num_2_item_dict;
  pcl::PCDWriter writer;
  std::vector<int> params;

  public:
  	save_images(): it_(nh){
  		kinect_pc_sub = nh.subscribe("/kinect2/hd/points", 1, &save_images::kinect_pc_cb, this);
  		mask_sub = it_.subscribe("/image_converter/output_video", 1, &save_images::mask_cb, this);
  		kinect_img_sub = it_.subscribe("/kinect2/hd/image_color", 1, &save_images::kinect_img_cb, this);

  		service = nh.advertiseService("save_images", &save_images::save_data_cb, this);

  		kinect_color_pc = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

      ros::param::get("num_2_item_dict",num_2_item_dict);

      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(100);
      params.push_back(cv::IMWRITE_PNG_COMPRESSION);
      params.push_back(1);
      params.push_back(cv::IMWRITE_PNG_STRATEGY);
      params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
      params.push_back(0);

      std::cout << "Constructor completed\n";

  	}

  	void kinect_pc_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
  		pcl::fromROSMsg(*cloud_msg, *kinect_color_pc);
  		new_pc = 1;
      if(waiting_for_new){
        std::cout << "new PC!\n";
      }
  	}
  	void kinect_img_cb(const sensor_msgs::ImageConstPtr& msg){
      kinect_color_raw = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  		new_raw = 1;
      if(waiting_for_new){
        std::cout<< "new IMAGE!\n";
      }


  	}
  	void mask_cb(const sensor_msgs::ImageConstPtr& msg){
  		kinect_color_mask = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  		new_mask = 1;
      if(waiting_for_new){
        std::cout << "new MASK!\n";	
      }
	}
  void set_dir_path(std::string path){
    dir_path = path;
  }

	bool save_data_cb(turntable::saveImages::Request &req,  turntable::saveImages::Response &res){
		/*uint16 item_number
		uint16 angle
		uint16 count
    string description
		---
		uint64 status
		*/
    std::cout << "Save Data request recieved: \n"<< 
           "item_number: "<< req.item_number << 
           ", angle: "<< req.angle <<
           ", count: "<< req.count << 
           ", description: " << req.description<<std::endl;
    std::string name_str;
    name_str.append(std::to_string(req.item_number));
    name_str.append("_");
    name_str.append(num_2_item_dict[std::to_string(req.item_number)]);
    name_str.append("_");
    name_str.append(std::to_string(req.angle));
    name_str.append("_");
    name_str.append(std::to_string(req.count));
    name_str.append("_");
    name_str.append(req.description);
    name_str.append("_");

    std::cout << "NAME :" << name_str<<std::endl;
    std::cout << "Waiting for new images\n";

    new_mask = 0;
    new_raw = 0;
    new_pc = 0;
    int sum = 0;
    while(sum != 3){
      waiting_for_new = 1;
      sum = new_mask + new_raw + new_pc;
      std::cout <<"new images: "<< sum << std::endl;
      ros::spinOnce();
      ros::Duration(.01).sleep();
    }
    waiting_for_new = 0;
    std::cout << "Writing data to: " << dir_path<<std::endl;
    
    const std::string cloudName = dir_path + name_str + "cloud.pcd";
    std::cout << "Cloud name: "<< cloudName<< std::endl;
    
    const std::string rawName = dir_path + name_str + "raw.jpg";
    const std::string maskName = dir_path + name_str + "mask.jpg";

    writer.writeBinary(cloudName, *kinect_color_pc);
    cv::imwrite(rawName, kinect_color_raw->image, params);
    cv::imwrite(maskName, kinect_color_mask->image, params);
    std::cout <<"Saving Complete\n";
    

		res.status = 1;
		return true;
	}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_images_server");    
    save_images si;

    
    // get time and date
    std::time_t now = time(0);
    std::tm *ltm = localtime(&now);
    std::string date_str;
    date_str.append( std::to_string(1900 + ltm->tm_year));
    date_str.append("_");
    date_str.append(std::to_string(1 + ltm->tm_mon));
    date_str.append("_");
    date_str.append(std::to_string(ltm->tm_mday));
    date_str.append("/");
    std::cout <<" date_str: " << date_str <<std::endl;


    // Get directory for data storage
    std::string dir_path = "/media/apark/Seagate Expansion Drive/turntable/";
    dir_path.append(date_str);
    boost::filesystem::path dir(dir_path);
    if(boost::filesystem::create_directory(dir)) {
      std::cout << "Created new Directory at " << dir_path<< std::endl;
    }
    si.set_dir_path(dir_path);
  
    ros::Rate loop_rate(10);
    while (ros::ok())
    {

      ros::spinOnce();
      loop_rate.sleep();
  }
    return 0;
}
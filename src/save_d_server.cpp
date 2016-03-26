#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include <tf/transform_listener.h>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <boost/thread/thread.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class ShelfMasker
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher pub_;
  ros::Subscriber kinect_sub_;
  ros::Subscriber masked_sub_;
  cv_bridge::CvImagePtr cv_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_filter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_color;
  pcl::PointCloud<pcl::PointXYZ>::Ptr kinect;
  pcl::PointCloud<pcl::PointXYZ>::Ptr corners;
  pcl::PointCloud<pcl::PointXYZ>::Ptr base;
  tf::TransformListener tf_;
  ros::ServiceServer save;

public:
  ShelfMasker() : it_(nh_)
  {
    // Initialize clouds
    kinect_color = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    kinect = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    corners = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    base = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    kinect_filter = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2/hd/image_color", 1, 
      &ShelfMasker::imageCb, this);
    // Subscribe to the kinect point cloud;
    kinect_sub_ = nh_.subscribe("/kinect2/hd/points", 1, &ShelfMasker::bin_cb, this);
    // Subscribe to geometry based filtered cloud
    masked_sub_ = nh_.subscribe("/bin_image", 1, &ShelfMasker::bin_filtered_cb, this);
    //  Publish resulting masked image
    pub_= it_.advertise("/shelf_masked", 1);
    // Set up service to save data  
    save = nh_.advertiseService("save_d", &ShelfMasker::save_data, this);
    // Set corner PCD
    ShelfMasker::get_corners();
    ShelfMasker::get_base();
  }

  void bin_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    //std::cout << "GRABBED kinect" << "\n" <<  std::flush;
    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *kinect_color);
    pcl::copyPointCloud(*kinect_color, *kinect);
  }

  void bin_filtered_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    //std::cout << "GRABBED filtered kinect" << "\n" <<  std::flush;
    // Convert to PCL data type
    pcl::fromROSMsg(*cloud_msg, *kinect_filter);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //std::cout << "GRABBED IMAGE" << "\n" <<  std::flush;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  // Get corner points based in bin frame
  void get_corners()
  {
    std::cout << "SET corners" << "\n";
    // Fill in the Cloud data
    corners->width    = 4;
    corners->height   = 1;
    corners->is_dense = false;
    corners->points.resize (corners->width * corners->height);
    // Hardcoded from shelf geometry
    float x[] = {0.01, 0.31, 0.31, 0.01};
    float y[] = {0.31, 0.31, 0.31, 0.31};
    float z[] = {0.0, 0.0, 0.37, 0.37};
    for (size_t i = 0; i < corners->points.size (); i++)
    {
      corners->points[i].x = x[i];
      corners->points[i].y = y[i];
      corners->points[i].z = z[i];
    }
  }

  // Get corner points based in bin frame
  void get_base()
  {
    std::cout << "SET base" << "\n";
    // Fill in the Cloud data
    base->width    = 4;
    base->height   = 1;
    base->is_dense = false;
    base->points.resize (base->width * base->height);
    // Hardcoded from shelf geometry
    float x[] = {0.01, 0.31, 0.31, 0.01};
    float y[] = {0.0, 0.0, 0.31, 0.31};
    float z[] = {0.38, 0.38, 0.38, 0.38};
    for (size_t i = 0; i < base->points.size (); i++)
    {
      base->points[i].x = x[i];
      base->points[i].y = y[i];
      base->points[i].z = z[i];
    }
  }

  bool save_data(std_srvs::Empty::Request&  req,
                 std_srvs::Empty::Response& res)
  {
    std::cout << "SAVING" << "\n" << std::flush;
    ros::Duration(3).sleep(); // sleep for TREE second
    double now = ros::Time::now().toSec();
    std::string savePath = "/home/harp/data/";
    savePath.append(std::to_string((int)now));
    // Save PCD file <filtered, world frame>
    std::string pcd_path = savePath;
    pcd_path.append(".pcd");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_filtered_world(new pcl::PointCloud<pcl::PointXYZRGB>);
    tf::StampedTransform t1;
    get_tf("world", "shelf");  
    kinect_filtered_world = transform_cloud (t1, kinect_filter);
    pcl::io::savePCDFileBinary(pcd_path, *kinect_filter);
    // Save jpg file <raw>
    std::string img_path = savePath;
    img_path.append(".jpg");
    cv::imwrite(img_path, cv_ptr->image);
    // Start text file
    std::ofstream datafile;
    std::string text_path = savePath;
    text_path.append(".txt");
    datafile.open(text_path);
    // Save Min Max Coordinates
    std::string currentBin;
    ros::param::get("/currentBin", currentBin);
    tf::StampedTransform t2;
    t2 = get_tf("world", currentBin);
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_world (new pcl::PointCloud<pcl::PointXYZ>);
    base_world = transform_cloud(t2, base);
    std::cout << "CORNERS: " "\n";
    for (size_t i=0; i<base->points.size(); i++)
    {
      std::cout << " " << base->points[i].x << " " << base->points[i].y << " " << base->points[i].z << "\n";
    }
    std::cout << "CORNERS WORLD: " "\n";
    for (size_t i=0; i<base_world->points.size(); i++)
    {
      std::cout << " " << base_world->points[i].x << " " << base_world->points[i].y << " " << base_world->points[i].z << "\n";
    }
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*base_world, min_p, max_p);
    datafile << min_p.x << " " << max_p.x << "\n";
    datafile << min_p.y << " " << max_p.y << "\n"; 
    datafile << min_p.z << "\n";   
    // Save TF Info
    tf::StampedTransform t3;
    t3 = get_tf("world", "kinect2_link");
    datafile << getTfMatrix(t3);
    datafile.close();
    return true;
  }

  void publish_image(cv::Mat);
  tf::StampedTransform get_bin_tf(std::string);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_corners (tf::StampedTransform);
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_kinect_cloud();
  cv_bridge::CvImagePtr get_image_ptr();
  //int save_data(std_msgs::Bool, std_msgs::Bool);
  tf::StampedTransform get_tf(std::string, std::string);
  Eigen::Matrix4f getTfMatrix(tf::StampedTransform);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transform_cloud (tf::StampedTransform t,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);   
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud (tf::StampedTransform,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr);
};

cv_bridge::CvImagePtr ShelfMasker::get_image_ptr()
{
  return cv_ptr;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ShelfMasker::get_kinect_cloud()
 {
  pcl::PointCloud<pcl::PointXYZ>::Ptr copy (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*kinect, *copy);
  return copy;
 }

void ShelfMasker::publish_image(cv::Mat image)
{
  std::cout << "Publishing" << "\n" <<  std::flush;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  pub_.publish(msg);
}

tf::StampedTransform ShelfMasker::get_bin_tf(std::string bin_frame)
{
  tf::StampedTransform transform;
  try{
    tf_.lookupTransform(bin_frame, "/kinect2_link",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return transform;  
}

tf::StampedTransform ShelfMasker::get_tf(std::string dest, std::string origin)
{
  tf::StampedTransform transform;
  try{
    tf_.lookupTransform(dest, origin,  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  return transform;  
}

Eigen::Matrix4f ShelfMasker::getTfMatrix(tf::StampedTransform t)
{
  float x = t.getOrigin().x();
  float y = t.getOrigin().y();
  float z = t.getOrigin().z();

  tf::Quaternion q;
  q = t.getRotation();

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::Affine3f A;
  pcl::getTransformation (x,y,z,roll,pitch,yaw,A);

  Eigen::Matrix4f M;
  M = A.matrix();
  return M;
}

// Transform scene (RGB) to new TF
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ShelfMasker::transform_cloud (tf::StampedTransform t,
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input)
{
  float x = t.getOrigin().x();
  float y = t.getOrigin().y();
  float z = t.getOrigin().z();

  tf::Quaternion q;
  q = t.getRotation();

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::Affine3f A;
  pcl::getTransformation (x,y,z,roll,pitch,yaw,A);

  Eigen::Matrix4f M;
  M = A.matrix();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::transformPointCloud (*input, *new_cloud, M);
  return new_cloud;
}

// Transform scene (RGB) to new TF
pcl::PointCloud<pcl::PointXYZ>::Ptr ShelfMasker::transform_cloud (tf::StampedTransform t,
  pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
  float x = t.getOrigin().x();
  float y = t.getOrigin().y();
  float z = t.getOrigin().z();

  tf::Quaternion q;
  q = t.getRotation();

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::Affine3f A;
  pcl::getTransformation (x,y,z,roll,pitch,yaw,A);

  Eigen::Matrix4f M;
  M = A.matrix();

  pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*input, *new_cloud, M);
  return new_cloud;
}

// Transform scene to align with shelf
pcl::PointCloud<pcl::PointXYZ>::Ptr ShelfMasker::transform_corners (tf::StampedTransform t)
{
  float x = t.getOrigin().x();
  float y = t.getOrigin().y();
  float z = t.getOrigin().z();

  tf::Quaternion q;
  q = t.getRotation();

  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  Eigen::Affine3f A;
  pcl::getTransformation (x,y,z,roll,pitch,yaw,A);

  Eigen::Matrix4f M;
  M = A.matrix();
  M = M.inverse().eval();

  pcl::PointCloud<pcl::PointXYZ>::Ptr new_corners (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*corners, *new_corners, M);
  return new_corners;
}

// Get image i,j values from from point cloud index
std::vector<cv::Point> get_image_points(std::vector<int> corner_ind, int cols)
{
  std::vector<cv::Point> pts;
  for (size_t i=0; i<corner_ind.size(); i++)
  {
    int j0 = (int) corner_ind[i] / cols;
    int i0 = (int) corner_ind[i] % cols;
    pts.push_back(cv::Point(i0, j0));    
  }
  return pts;
}

// Get closest 3D points to expected corner points
std::vector<int> get_closest_points(pcl::PointCloud<pcl::PointXYZ>::Ptr kinect,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr corners)
{
  pcl::KdTree<pcl::PointXYZ>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tree_->setInputCloud(kinect);

  std::vector<int> ind;
  for (size_t i=0; i<corners->points.size(); i++)
  {
    std::vector<int> p_ind (1);
    std::vector<float> dist (1);
    tree_->nearestKSearch(pcl::PointXYZ(corners->points[i].x, corners->points[i].y, corners->points[i].z), 1, p_ind, dist);
    ind.push_back(p_ind[0]);
  }
  return ind;
}

// Get min and max boundaries of mask
std::vector<int> getBounds (std::vector<cv::Point> pts, int rows, int cols)
{  
  std::vector<int> p_ind (pts.size());
  p_ind[0] = cols; p_ind[1] = 0;
  p_ind[2] = rows; p_ind[3] = 0;
  for (size_t i=0; i<pts.size();i++)
  {
    if (pts[i].x < p_ind[0]) p_ind[0] = pts[i].x;
    if (pts[i].x > p_ind[1]) p_ind[1] = pts[i].x;
    if (pts[i].y < p_ind[2]) p_ind[2] = pts[i].y;
    if (pts[i].y > p_ind[3]) p_ind[3] = pts[i].y;
  }
  return p_ind;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_masker");

  ShelfMasker sm;
  
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    // Get TF from kinect to bin
    tf::StampedTransform t;
    std::string currentBin;
    if (ros::param::get("/currentBin", currentBin))
    {
      std::cout << "TF LOOKUP" << "\n";
      t = sm.get_bin_tf(currentBin);
      //std::cout << "GRABBED TF" << "\n";
    }

    // Transform corners into kinect frame
    if ((sm.get_kinect_cloud()->points.size() != 0) && (sm.get_image_ptr() != NULL))
    {
      std::cout << "IN DA LOOP" << "\n";

      pcl::PointCloud<pcl::PointXYZ>::Ptr corners (new pcl::PointCloud<pcl::PointXYZ>);
      corners = sm.transform_corners(t);

      // Get closest point indicies to each 3D corner point
      std::vector<int> corner_ind = get_closest_points(sm.get_kinect_cloud (), corners);
      
      cv::Mat image = sm.get_image_ptr()->image;

      // Convert from points to indicies
      std::vector<cv::Point> pts = get_image_points(corner_ind, image.cols);

      const cv::Point* elementPoints[1] = { &pts[0] };
      int numPoints = pts.size();
      cv::Size s = image.size();
      cv::Mat mMask = cv::Mat::zeros(s, CV_8U);
      mMask.setTo(0);
      // Create mask from ROI points
      cv::fillPoly(mMask, elementPoints, &numPoints, 1, cv::Scalar(255));
      
      // Apply mask to image
      cv::Mat outMask = cv::Mat::zeros(s, CV_8UC3);
      cv::bitwise_and(image, image, outMask,mMask);

      // Get min and max of masked shelf
      std::vector<int> bounds = getBounds(pts, image.rows, image.cols);

      // Image for CNN must be 185 x 195
      cv::Rect roi(bounds[0], bounds[2], bounds[1]-bounds[0], bounds[3]-bounds[2]);
      cv::Mat image_roi = outMask(roi);
      cv::Size size(185,195);
      cv::Mat downsized_image;
      cv::resize(image_roi,downsized_image,size);

      sm.publish_image(downsized_image);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
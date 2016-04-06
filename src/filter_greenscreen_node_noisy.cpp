#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/opencv.hpp"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int H_low;
  int H_high;
  int S_low;
  int S_high;
  int V_low;
  int V_high;
  float x_window_origin;
  float y_window_origin;
  float x_window_size;
  float y_window_size;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    std::cout << "inside constructor\n";
    image_sub_ = it_.subscribe("/kinect2/hd/image_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    H_low = 50;
    H_high = 73;

    S_low = 109;
    S_high = 255;

    V_low = 0;
    V_high = 255;

    x_window_origin = .17;
    y_window_origin = .33;
    x_window_size = .5;
    y_window_size = .66;

    cvNamedWindow("Image window", 2);
    cvNamedWindow ("trackbar", 2 );
    cvNamedWindow("prefiltered image",2);
    cvSetMouseCallback("Image Window", ImageConverter::mouseHandler,0);
    cvCreateTrackbar( "H_low", "trackbar", &H_low, 255, 0 );
    cvCreateTrackbar( "H_high", "trackbar", &H_high, 255, 0 );
    cvCreateTrackbar( "S_low", "trackbar", &S_low, 255, 0 );
    cvCreateTrackbar( "S_high", "trackbar", &S_high, 255, 0 );
    cvCreateTrackbar( "V_low", "trackbar", &V_low, 255, 0 );
    cvCreateTrackbar( "V_high", "trackbar", &V_high, 255, 0 );
    //cvCreateTrackbar( "x_pos(pct)", "trackbar", &x_window, 50, 0);
    //cvCreateTrackbar( "y_pos(pct)", "trackbar", &y_window, 33, 0);
  }

  ~ImageConverter()
  {
    cv::destroyWindow("Image window");
  }

  static void mouseHandler(int event, int x, int y, int flags, void* param){
    std::cout << "mouse callback: "<< event<<"\n";
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
      printf("Left button down with CTRL pressed\n");
      break;
    case CV_EVENT_LBUTTONUP:
        printf("Left button up\n");
        break;
    }
  }

  void update_window_params(){
    ros::param::get("/x_window_origin", x_window_origin);
    ros::param::get("/y_window_origin", y_window_origin);
    ros::param::get("/x_window_size", x_window_size);
    ros::param::get("/y_window_size", y_window_size);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr, hsv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      hsv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    update_window_params();

    cv::Mat PreFilterImage;
    cv::Mat HSVImage;
    cv::Mat ThreshImage;
	cv::Mat blobImage;
    
    //Transform the colors into HSV

    int rows = cv_ptr->image.rows;
    int cols = cv_ptr->image.cols;
    //std::cout << "rows, cols: "<< rows <<  ", "<< cols<< std::endl;
	cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8U);
    cv::Mat roi_mask = cv::Mat::zeros(rows, cols, CV_8U); // all 0
	cv::Mat cc_mask = cv::Mat::zeros(rows, cols, CV_8U);
    roi_mask(cv::Rect( int(x_window_origin*float(cols)) , 
                       int(y_window_origin*float(rows)) ,
                       int(x_window_size*float(cols)) , 
                       int(y_window_size*float(rows))   ) ) = 1;

    cv_ptr->image.copyTo(PreFilterImage, roi_mask);

    cv::cvtColor(PreFilterImage,HSVImage,CV_BGR2HSV);
    //for blue
    cv::inRange(HSVImage,cv::Scalar(H_low,S_low,V_low),cv::Scalar(H_high,S_high,V_high),mask);
    //std::cout << "assign to img\n";
    mask = 255 - mask;
	PreFilterImage.copyTo(ThreshImage, mask);
    hsv_ptr->image = ThreshImage;
	/*
    // Do blob detection
    // set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 50.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 100.0f;
	params.maxArea = 200000.0f;
	// ... any other params you don't want default value

	// set up and create the detector using the parameters
	cv::SimpleBlobDetector blob_detector(params);
	// or cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params)

	// detect!
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat blobMask = cv::Mat::zeros(rows, cols, CV_8U); // all 0
	blob_detector.detect(ThreshImage, keypoints, blobMask);
	blobMask = 255 - blobMask;
	ThreshImage.copyTo(blobImage, blobMask);

	// extract the x y coordinates of the keypoints: 

	for (int i=0; i<keypoints.size(); i++){
		float X = keypoints[i].pt.x; 
		float Y = keypoints[i].pt.y;
		
	}
	cv::drawKeypoints(blobImage,keypoints,blobImage,cv::Scalar::all(-1));
	
	*/

    // do contour filling


	std::vector<std::vector<cv::Point> > contours;
  	std::vector<cv::Vec4i> hierarchy;
	// Find contours
	cv::findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	for(int idx = 0; idx < contours.size(); idx++){	
		cv::drawContours( cc_mask, contours, idx, cv::Scalar(255, 0, 0), CV_FILLED, 8);
	}
	ThreshImage.copyTo(blobImage, cc_mask);

    //std::cout << " after\n";
    
    // Output modified video stream
    image_pub_.publish(hsv_ptr->toImageMsg());
    //std::cout << "publishing new image\n";
    cv::imshow("prefiltered image", PreFilterImage);
    cv::imshow("Image window", blobImage);
    cv::waitKey(3);
    //std::cout << "updated window"<<std::endl;
    
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

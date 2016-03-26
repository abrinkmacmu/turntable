#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
  int x_window;
  int y_window;
  
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

    x_window = 17;
    y_window = 33;

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
    cvCreateTrackbar( "x_pos(pct)", "trackbar", &x_window, 50, 0);
    cvCreateTrackbar( "y_pos(pct)", "trackbar", &y_window, 33, 0);
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
    cv::Mat PreFilterImage;
    cv::Mat HSVImage;
    cv::Mat ThreshImage;
    cv::Mat mask;
    //Transform the colors into HSV

    int rows = cv_ptr->image.rows;
    int cols = cv_ptr->image.cols;
    //std::cout << "rows, cols: "<< rows <<  ", "<< cols<< std::endl;
    cv::Mat roi_mask = cv::Mat::zeros(rows, cols, CV_8U); // all 0
    roi_mask(cv::Rect( int(float(x_window)/100.0*float(cols)) , int(float(y_window)/100.0*float(rows)) ,int(float(cols)/2.0) , int(2.0*float(rows)/3.0) )) = 1;

    cv_ptr->image.copyTo(PreFilterImage, roi_mask);

    cv::cvtColor(PreFilterImage,HSVImage,CV_BGR2HSV);
    //for blue
    cv::inRange(HSVImage,cv::Scalar(H_low,S_low,V_low),cv::Scalar(H_high,S_high,V_high),mask);
    //std::cout << "assign to img\n";
    mask = 255 - mask;
    PreFilterImage.copyTo(ThreshImage, mask);
    hsv_ptr->image = ThreshImage;
    //std::cout << " after\n";
    
    // Output modified video stream
    image_pub_.publish(hsv_ptr->toImageMsg());
    //std::cout << "publishing new image\n";
    cv::imshow("prefiltered image", PreFilterImage);
    cv::imshow("Image window", ThreshImage);
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
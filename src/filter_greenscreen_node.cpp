#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "opencv2/opencv.hpp"
#include <cmath>

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
  int N_hull;
  int do_convex_hull;
  float x_window_origin;
  float y_window_origin;
  float x_window_size;
  float y_window_size;
int invert_filter;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    std::cout << "inside constructor\n";
    image_sub_ = it_.subscribe("/kinect2/hd/image_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
	/*
    H_low = 50;
    H_high = 73;

    S_low = 109;
    S_high = 255;

    V_low = 0;
    V_high = 255;
	*/

	H_low = 2;
    H_high = 40;

    S_low = 0;
    S_high = 255;

    V_low = 0;
    V_high = 255;
	N_hull = 2;
    do_convex_hull = 1;

    x_window_origin = .17;
    y_window_origin = .33;
    x_window_size = .5;
    y_window_size = .66;

    cvNamedWindow("Image window", 2);
    cvNamedWindow ("trackbar", 2 );
    cvNamedWindow("prefiltered image",2);
	cvNamedWindow("drawing", 2);
	cvNamedWindow("HSV Thresh", 2);
    cvSetMouseCallback("Image Window", ImageConverter::mouseHandler,0);
    cvCreateTrackbar( "H_low", "trackbar", &H_low, 255, 0 );
    cvCreateTrackbar( "H_high", "trackbar", &H_high, 255, 0 );
    cvCreateTrackbar( "S_low", "trackbar", &S_low, 255, 0 );
    cvCreateTrackbar( "S_high", "trackbar", &S_high, 255, 0 );
    cvCreateTrackbar( "V_low", "trackbar", &V_low, 255, 0 );
    cvCreateTrackbar( "V_high", "trackbar", &V_high, 255, 0 );
	cvCreateTrackbar( "Hull_size", "trackbar", &N_hull, 2, 0);
    cvCreateTrackbar( "doConvexHull", "trackbar", &do_convex_hull, 1,0);
	cvCreateTrackbar("invertFilter", "trackbar", &invert_filter, 1,0);
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
cv::Mat final_img;
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
   if(invert_filter){
    	mask = 255 - mask;
    }
	/*std::cout << "denoising...\n";
	cv::fastNlMeansDenoising(mask, mask);
	std::cout << "Finishedh\n";*/
	PreFilterImage.copyTo(ThreshImage, mask);
   if(do_convex_hull){	
	std::vector<std::vector<cv::Point> > contours;
	//std::vector<cv::Point> contours;
    std::vector<cv::Vec4i> hierarchy;

    /// Find contours
   cv::findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

   /// Find the convex hull object for each contour
   std::vector<std::vector<cv::Point> >hull( contours.size() );
	//std::vector<cv::Point> hull( contours.size() );
   double largest_hull = 0;
   int largest_hull_i = 0;
   double sec_largest_hull = 0;
   int sec_largest_hull_i = 0;
	double third_hull =0;
	int third_hull_i = 0;
   std::vector<cv::Point> areaContour;
	double area = 0;
   for( int i = 0; i < contours.size(); i++ ){ 
		
		cv::convexHull( cv::Mat(contours[i]), hull[i], false ); 
		

		//calc area
		area = 0;
		for (int j = 0; j < hull[i].size(); j++){
			int next_j = (j+1)%(hull[i].size());
			double dX   = hull[i][next_j].x - hull[i][j].x;
			double avgY = (hull[i][next_j].y + hull[i][j].y)/2;
			area += dX*avgY;  // This is the integration step.
		}

		//std::cout << i << ", size: "<< contours[i].size()<<", area: " << area<< std::endl; 

		if(std::abs(area) > largest_hull){
			third_hull = sec_largest_hull;
			third_hull_i = sec_largest_hull_i;
	 		sec_largest_hull = largest_hull;
			sec_largest_hull_i = largest_hull_i;
			largest_hull = std::abs(area) ;
		    largest_hull_i = i;
		}else if(std::abs(area) > sec_largest_hull){
			third_hull = sec_largest_hull;
			third_hull_i = sec_largest_hull_i;
			sec_largest_hull = std::abs(area) ;
		    sec_largest_hull_i = i;
		}else if(std::abs(area) > third_hull){
			third_hull = std::abs(area);
			third_hull_i = i;
		}
	}	
	std::cout << "sizes of largest hulls: " << largest_hull << ", " << sec_largest_hull << std::endl;

   /// Draw contours + hull results
   cv::Mat drawing = cv::Mat::zeros( ThreshImage.size(), CV_8U );
   cv::Mat drawing_visual = cv::Mat::zeros(ThreshImage.size(), CV_8UC3);
  // for( int i = 0; i< contours.size(); i++ ){
	int i;
	if (N_hull == 0){
		i = largest_hull_i;
	}else if(N_hull == 1){
		i = sec_largest_hull_i;
	}else if(N_hull == 2){
		i = third_hull_i;
	}
        cv::Scalar color = cv::Scalar( 255, 0, 0 );
		cv::Scalar color_2 = cv::Scalar(255);
        //cv::drawContours( drawing_visual, hull, i, color, 1, 8, hierarchy, 2, cv::Point() );
        cv::drawContours( drawing, hull, i, color_2, CV_FILLED, 8);
    //  }
	
	
	PreFilterImage.copyTo(final_img, drawing);

	hsv_ptr->image =final_img;
    //std::cout << " after\n";
    }else{
       hsv_ptr->image = ThreshImage;
	final_img = ThreshImage;
	}
    
    // Output modified video stream
    image_pub_.publish(hsv_ptr->toImageMsg());
    //std::cout << "publishing new image\n";
    cv::imshow("prefiltered image", PreFilterImage);
	cv::imshow("HSV Thresh", ThreshImage);
	//cv::imshow("drawing", drawing_visual);
    cv::imshow("Image window", final_img);
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

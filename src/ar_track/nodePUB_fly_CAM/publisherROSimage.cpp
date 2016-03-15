#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <arpa/inet.h>
//#include <boost/endian/conversion.hpp>
#include <FlyCap.h>

using namespace std;
using namespace cv;
 
int main(int argc, char** argv)
{
   ros::init(argc, argv, "FLYCAM_image_publisher");// ROS node
   ros::NodeHandle nh;


   image_transport::ImageTransport it(nh);
   image_transport::Publisher pub = it.advertise("camera/image", 1);
  
	FlyCap FCapture;

	FCapture.init();	//inizializzo la webcam e parte il ciclo di cattura
	Image rgbImg;


   ros::Rate loop_rate(100);//RATE

    while (nh.ok()) 
	{
		FCapture.flyRaw();
   	 
		//cout<<"before of take the image"<<endl;

		 //cv::Mat source_image = FCapture.getImageOpencv();// take image from Camera
   	 //cv::waitKey(30);//RATE
   
		//if(source_image.empty()){cout<<"emptyIMAGE"<<endl; continue;}

		cout<<"before sensor_msgs with cv_bridge"<<endl;

		sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>(); /*cv_bridge::CvImage(std_msgs::Header(), "bgr8", source_image).toImageMsg();*/
	//puntatore al template chiamando il costruttore del deplate


		if(htonl(47)==47){msg->is_bigendian = 1; }else{msg->is_bigendian = 0;}

		FCapture.getImageFlyCapRGB8(rgbImg);

		msg->header.frame_id =  "G_optical_frame";
		msg->height = rgbImg.GetRows();//source_image.row;
		msg->width = rgbImg.GetCols();//source_image.col;
		msg->encoding = "rgb8";
		
		msg->step = rgbImg.GetStride();

		size_t size = msg->step*msg->height;
		msg->data.resize(size);

		memcpy((char*)(&msg->data[0]), rgbImg.GetData(), size);

		cout<<"post "<<endl;

		pub.publish(msg);
	

  		 ros::spinOnce();
   	 loop_rate.sleep();
	}

	FCapture.closeVideo();

}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
//
//		Converting OpenCV images to ROS image messages
//
//
//



/*
		 class CvImage
   {
      sensor_msgs::ImagePtr toImageMsg() const;
    
      // Overload mainly intended for aggregate messages that contain
      // a sensor_msgs::Image as a member.
      void toImageMsg(sensor_msgs::Image& ros_image) const;
    };



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
     
  public:
     ImageConverter()
       : it_(nh_)
     {
       // Subscrive to input video feed and publish output video feed
       image_sub_ = it_.subscribe("/camera/image_raw", 1, 
         &ImageConverter::imageCb, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);
   
       cv::namedWindow(OPENCV_WINDOW);
     }
   
     ~ImageConverter()
     {
       cv::destroyWindow(OPENCV_WINDOW);
     }
   
     void imageCb(const sensor_msgs::ImageConstPtr& msg)
     {
       cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
   
       // Draw an example circle on the video stream
       if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
         cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
   
       // Update GUI Window
       cv::imshow(OPENCV_WINDOW, cv_ptr->image);
       cv::waitKey(3);
       
       // Output modified video stream
       image_pub_.publish(cv_ptr->toImageMsg());
     }
  };
  
  int main(int argc, char** argv)
  {
     ros::init(argc, argv, "image_converter");
     ImageConverter ic;
     ros::spin();
     return 0;
  }

















*/




















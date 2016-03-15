#include <ros/ros.h>
#include <image_transport/image_transport.h> // includes everything we need to publish and subscribe to images.

//These headers will allow us to display images using OpenCV's simple GUI capabilities.
 #include <opencv2/highgui/highgui.hpp>
 #include <cv_bridge/cv_bridge.h>
   
/*
This is the callback function that will get called when a new image has arrived on the "camera/image" topic. Although the image may have been sent in some arbitrary transport-specific message type, notice that the callback need only handle the normal sensor_msgs/Image type. All image encoding/decoding is handled automagically for you.

The body of the callback. We convert the ROS image message to an OpenCV image with BGR pixel encoding, then show it in a display window.


*/

 void imageCallback(const sensor_msgs::ImageConstPtr& msg)
 {
    try
    {
     cv::imshow("view", cv_bridge::toCvShare(msg, "rgb8")->image);
     cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
     }
 }
  
int main(int argc, char **argv)
{
   ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cv::namedWindow("view");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
}



//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
//		
//		CONVERTING 
//
//---->>>>>	ROS image messages to OpenCV images
//
//		CvBridge defines a CvImage type containing an OpenCV image, 
//		its encoding and a ROS header. 
//		CvImage contains exactly the information sensor_msgs/Image does, 
//		so we can convert either representation to the other. CvImage class format:


/*

namespace cv_bridge {

class CvImage
{
   public:
   	  std_msgs::Header header;
        std::string encoding;
        cv::Mat image;
    };
 
 	typedef boost::shared_ptr<CvImage> CvImagePtr;
   typedef boost::shared_ptr<CvImage const> CvImageConstPtr;

/*
When converting a ROS sensor_msgs/Image message into a CvImage, 
CvBridge recognizes two distinct use cases:

We want to modify the data in-place. 
We have to make a copy of the ROS message data.
We won't modify the data. 
We can safely share the data owned by the ROS message instead of copying.
CvBridge provides the following functions for converting to CvImage:
*/



/*
toCvCopy creates a copy of the image data from the ROS message,
 even when the source and destination encodings match. 
However, you are free to modify the returned CvImage.



	// Case 1: Always copy, returning a mutable CvImage
   CvImagePtr toCvCopy(const sensor_msgs::ImageConstPtr& source,
                        const std::string& encoding = std::string());
   CvImagePtr toCvCopy(const sensor_msgs::Image& source,
                        const std::string& encoding = std::string());
    

toCvShare will point the returned cv::Mat at the ROS message data, 
avoiding a copy, if the source and destination encodings match. 
As long as you hold a copy of the returned CvImage, 
the ROS message data will not be freed. If the encodings do not match, 
it will allocate a new buffer and perform the conversion. 
You are not permitted to modify the returned CvImage, 
as it may share data with the ROS image message, 
which in turn may be shared with other callbacks. 
Note: the second overload of toCvShare is more convenient 
when you have a pointer to some other message type 
(e.g. stereo_msgs/DisparityImage) 
that contains a sensor_msgs/Image you want to convert.


If no encoding (or rather, the empty string) is given, 
the destination image encoding will be the same as the image message encoding. 
In this case 

toCvShare is guaranteed to not copy the image data. 

 
Image encodings can be any one of the following OpenCV image encodings:


   // Case 2: Share if possible, returning a const CvImage
   CvImageConstPtr toCvShare(const sensor_msgs::ImageConstPtr& source,
                              const std::string& encoding = std::string());
   CvImageConstPtr toCvShare(const sensor_msgs::Image& source,
                             const boost::shared_ptr<void const>& tracked_object,
                             const std::string& encoding = std::string());


}

*/



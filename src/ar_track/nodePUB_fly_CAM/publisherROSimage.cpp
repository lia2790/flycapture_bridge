#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <arpa/inet.h>
//#include <boost/endian/conversion.hpp>
#include <FlyCap.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace cv;
 
int main(int argc, char** argv)
{
 
	//inizializzazione nodo ROSPTGREY
	ros::init(argc, argv, "FLYCAM_image_publisher");// ROS node
   ros::NodeHandle nh;

	//per pubblicare sul topic l'immagine prelevata dalla camera
   image_transport::ImageTransport it(nh);
   image_transport::Publisher pub = it.advertise("camera/image", 1);
  
	
	//per pubblicare sul topic le camera_info (matrice intrinseca, etc ) 
	//image_transport::ImageTransport it_f(nh);
	ros::Publisher pub_f = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info",1);

	// get current CameraInfo data
   sensor_msgs::CameraInfoPtr cam_info = boost::make_shared<sensor_msgs::CameraInfo>();
		
	// CameraInfo
		cam_info->header.frame_id = "G_optical_frame"; 
      cam_info->height = 2048;
      cam_info->width  = 2048;
		cam_info->distortion_model = "plumb_bob";
		cam_info->D.push_back(-0.084581);
		cam_info->D.push_back(0.178136);
		cam_info->D.push_back(-0.001719);
		cam_info->D.push_back(-0.000675);
		cam_info->D.push_back(0.000000);
//D è un std_vector
//distortion vector -0.084581 0.178136 -0.001719 -0.000675 0.000000

		cam_info->K[0] = 2372.888903;
		cam_info->K[1] = 0.000000;
		cam_info->K[2] = 1039.133464;
		cam_info->K[3] = 0.000000;
		cam_info->K[4] = 2365.795545;
		cam_info->K[5] = 994.137776;
		cam_info->K[6] = 0.000000;
		cam_info->K[7] = 0.000000;
		cam_info->K[8] = 1.000000;

//2372.888903 0.000000 1039.133464
//0.000000 2365.795545 994.137776
//0.000000 0.000000 1.000000



		cam_info->R[0] = 1.000000;
		cam_info->R[1] = 0.000000;
		cam_info->R[2] = 0.000000;
		cam_info->R[3] = 0.000000;
		cam_info->R[4] = 1.000000;
		cam_info->R[5] = 0.000000;
		cam_info->R[6] = 0.000000;
		cam_info->R[7] = 0.000000;
		cam_info->R[8] = 1.000000;
		
//identità



		cam_info->P[0] = 2358.461670;
		cam_info->P[1] = 0.000000;
		cam_info->P[2] = 1037.212841;
		cam_info->P[3] = 0.000000;
		cam_info->P[4] = 0.000000;
		cam_info->P[5] = 2351.061768;
		cam_info->P[6] = 991.213209;
		cam_info->P[7] = 0.000000;
		cam_info->P[8] = 0.000000;
		cam_info->P[9] = 0.000000;
		cam_info->P[10] = 1.000000;
		cam_info->P[11] = 0.000000;
			

//2358.461670 0.000000 1037.212841 0.000000
//0.000000 2351.061768 991.213209 0.000000
//0.000000 0.000000 1.000000 0.000000



		cam_info->binning_x = 0;
		cam_info->binning_y = 0;
		cam_info->roi.width  = 0;
		cam_info->roi.height = 0;
		cam_info->roi.x_offset = 0;
		cam_info->roi.y_offset = 0;
		cam_info->roi.do_rectify = 0;




		//classe del driver
		FlyCap FCapture;
		FCapture.init();
		Image rgbImg;






		//creiamo un legame fra i sistemi di riferimento
		std::string stringaFrameIdPadre = "link_fisico";
		std::string stringaFrameIdFiglio = "G_optical_frame";


		tf::TransformBroadcaster tf_broadcaster; 



		double px = 0;//p.translation[0]/100.0;
		double py = 0;//p.translation[1]/100.0;
		double pz = 0.03;//metri   //p.translation[2]/100.0;
		double qx = 0;//p.quaternion[1];
		double qy = 0;//p.quaternion[2];
		double qz = 0;//p.quaternion[3];
		double qw = 1;//p.quaternion[0];


		tf::Quaternion rotazione(qx,qy,qz,qw);
      tf::Vector3 traslazione(px,py,pz);

      tf::Transform trasformazione(rotazione, traslazione);


		
		//tf::StampedTransform camLinkTocamOpt(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
		//tf_broadcaster.sendTransform(camLinkTocamOpt);//pubblicare la trasf su TF(è un pacchetto con un topic)
	



   	ros::Rate loop_rate(100);//RATE, non metterlo al massimo sennò occupo tutta la CPU ( no good )

   	while (nh.ok()) 
		{
			FCapture.flyRaw();
   	

			sensor_msgs::ImagePtr img = boost::make_shared<sensor_msgs::Image>(); 
	     /*cv_bridge::CvImage(std_msgs::Header(), "bgr8", source_image).toImageMsg();*/
		  //puntatore al template chiamando il costruttore del deplate

			//riempimento a mano del pacchetto da inviare sul topic : "camera/camera_info"
			//pacchetto che andrà su talun buffer : sensor_msgs::ImagePtr msg
			if(htonl(47)==47){img->is_bigendian = 1; }else{img->is_bigendian = 0;}

			FCapture.getImageFlyCapRGB8(rgbImg);

			img->header.frame_id =  "G_optical_frame";
			img->height = rgbImg.GetRows();//source_image.row by direct_flycap;
			img->width = rgbImg.GetCols();//source_image.col by direct_flycap;
			img->encoding = "rgb8";
			img->step = rgbImg.GetStride();
			size_t size = (img->step)*(img->height);
			img->data.resize(size);

			memcpy((char*)(&img->data[0]), rgbImg.GetData(), size);


			tf::StampedTransform camLinkTocamOpt(trasformazione, ros::Time::now(), stringaFrameIdPadre, stringaFrameIdFiglio);
    	
			tf_broadcaster.sendTransform(camLinkTocamOpt);//pubblicare la trasf su TF(è un pacchetto con un topic)
	
		

			pub.publish(img);
			pub_f.publish(*cam_info);

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




















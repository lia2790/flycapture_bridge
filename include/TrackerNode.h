#include <fstream>
#include <sstream>
#include <iostream>
#include <math.h>
#include <cstdio>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "markerdetector.h"
#include "cvdrawingutils.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <object_detection/start.h>
#include <object_detection/stop.h>
#include <ros/package.h>

#include "FlyCap.h"//for point grey camera

using namespace std;
using namespace cv;
using namespace mid;
using namespace FlyCapture2;

class TrackerNode
{
	public:
	
		//ROS node
		ros::NodeHandle n;

		ros::ServiceServer start_serv;
	   ros::ServiceServer stop_serv;
		bool action;//flag of activation

//-----> OPENCV USING

		//obj track, image processing
		std::string filename_camera;
		double size_marker;
		mid::CameraParameters CamParam;//use the pinhole camera model		
		//camera value of intrinsics matrix obtained with opencv calibration example with chessboard
		MarkerDetector MDetector; //marker to find in image processing
		vector< Marker > Markers;//marker candidate		
		cv::VideoCapture vreader; //streamvideo
		cv::Mat Image;//input image for show it
		cv::Mat ImageCopy;
		
		//create a gui
		double ThresParam1, ThresParam2;
		int iThresParam1, iThresParam2;

//------------------------------------------> FLYCAPTURE2 by POINT GREY

		FlyCap FCapture;
		int use_camera;

//__________________________________________>METHODS
//
		TrackerNode();
		~TrackerNode() {}


		void go(bool set); //call it in your program for starting track algorithm  
/*
		void openWithVideo0();
		void openWithFlyCap();
*/	

		//callback function
		bool startTrack(object_detection::start::Request &req, object_detection::start::Response &res);
		bool stopTrack(object_detection::stop::Request &req, object_detection::stop::Response &res);

		
		//threshold a runtime 
		static void cvTrackBarEventsStatic(int pos, void *that);
		void cvTrackBarEvents(int pos, TrackerNode *pr);







		void startStreamThread();





};  

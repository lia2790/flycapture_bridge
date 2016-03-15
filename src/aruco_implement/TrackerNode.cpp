#include <iostream> 
#include "TrackerNode.h"

using namespace std;
using namespace cv;

TrackerNode::TrackerNode()
{
	n = ros::NodeHandle("ObjectTracker");//create ROS node

	//create the service
   start_serv =  n.advertiseService("Start",&TrackerNode::startTrack,this);
	stop_serv =  n.advertiseService("Stop",&TrackerNode::stopTrack, this);

	//take the param from file
	n.param<std::string>("filename_camera", filename_camera, " ");
	n.param<double>("size_marker", size_marker, -1);
	n.param<int>("use_camera",use_camera, 0);// 0 for grey point camera / 1 for usb camera with video0 

	//iniz the camera parameters
	std::string pathFile = ros::package::getPath("object_detection") +"/"+ filename_camera;
	CamParam.readFromXMLFile(pathFile); 
	
	//for start without the obj track
	action = false;
}


///********************************************************************
///		RESEARCH MARKERS AND VISUAL FEEDBACK
///********************************************************************


//calling in the loop
void TrackerNode::go(bool set)
{

	//cout<<"I'm going"<<endl;

	//choose camera and
	//get image from video for image processing
	if(use_camera)
		vreader >> Image;
	else
	{
		cout<<"i try to take a image from camera"<<endl;
		Image = FCapture.getImageOpencv();

		cout<<"return to take a image for processing"<<endl;
	//	cv::imshow("image", FCapture.image_for_OPENCV);
		//cv::imshow("imageTAKE",Image);//for test and done   well */
	}

	// Create a new matrix to hold the gray image
      // Mat ImageCop(Image);don't work
 	Mat ImageCop;
       // convert RGB image to gray
   cvtColor(Image, ImageCop, COLOR_BGR2GRAY );//work but i could want the colors!!!!!
	
	//cvtColor(ImageCop,ImageCop,COLOR_GRAY2BGR);//don't work
	//ImageCop = Image.clone();	
	//don't work fail encoding
	//cv::Mat ImageCop;
	//cv::Mat ImageCop = Image.clone();//sporca l 'immagine
	//Image.copyTo(ImageCop);
	//ImageCopy = Image; //puntatore alla stessa struttura
	//cv::imshow("ImageBefore",Image);

	// function for detection markers on image
	MDetector.detect(Image, Markers, CamParam, size_marker);
	//cv::imshow("Imag",Image);
	
	if(set)
	{	//Try to draw the result
		for (unsigned int i = 0; i < Markers.size(); i++) 
		{
			cout << Markers[i] << endl;//print the value of sigle marker
      	Markers[i].draw(ImageCop, Scalar(0, 0, 255), 2);
		}

		// draw a 3d cube in each marker if there is 3d info
		if (CamParam.isValid() && size_marker != -1)
			for (unsigned int i = 0; i < Markers.size(); i++) 
      	{   
				CvDrawingUtils::draw3dCube(ImageCop, Markers[i], CamParam);
				CvDrawingUtils::draw3dAxis(ImageCop, Markers[i], CamParam);
			}   

		//cv::imshow("show",Image);
		// show input with augmented information
		cv::imshow("in", ImageCop);
		// show also the internal image resulting from the threshold operation
		cv::imshow("thres", MDetector.getThresholdedImage());
		int key = waitKey(1);
	}
}

//************************************************************



///*************************************************************
///		THRESHOLD SETTING A RUN-TIME
///*************************************************************

void TrackerNode::cvTrackBarEventsStatic(int pos, void *that)
{
	TrackerNode &th = *(TrackerNode *)(that);
	th.cvTrackBarEvents(pos, (TrackerNode *)(that));
}



void TrackerNode::cvTrackBarEvents(int pos, TrackerNode *pr) 
{
	Mat TheInputImageCopy;

	if (pr->iThresParam1 < 3)			pr->iThresParam1 = 3;
   if (pr->iThresParam1 % 2 != 1)  	pr->iThresParam1++;
   if (pr->ThresParam2 < 1)			pr->ThresParam2 = 1;
	
  
	pr->ThresParam1 = pr->iThresParam1;
	pr->ThresParam2 = pr->iThresParam2;
 
	pr->MDetector.setThresholdParams(pr->ThresParam1, pr->ThresParam2);

	//cout<<pr->Image.size()<<endl;    

	pr->Image.copyTo(TheInputImageCopy);

//	cv::imshow("in", TheInputImageCopy);  
//	cv::imshow("thres", pr->MDetector.getThresholdedImage());
	
}

///************************************************************************



///************************************************************************
///		START and STOP SERVICE IN ROS NODE
///*************************************************************************
/*
void TrackerNode::openWithVideo0()
{
	vreader.open(0);//open the stream video

	if (vreader.isOpened()) 
	{
		vreader.grab();
		vreader.retrieve(Image);
	}
	else {cout<<"error open video"<<endl;}

	vreader >> Image;
}
*/
/*
void TrackerNode::openWithFlyCap()
{
	FCapture.init();
	Image = FCapture.getImageOpencv();
}
*/
bool TrackerNode::startTrack(object_detection::start::Request &req, object_detection::start::Response &res)
{
	//before to call function go() for image processing

	action = true;//flag for starting track

	//create a window for show image captured
	cv::namedWindow("in", CV_WINDOW_NORMAL);
	cv::namedWindow("thres", CV_WINDOW_NORMAL);

	//create threshold_bar for choose a runtime the value
	MDetector.getThresholdParams(ThresParam1, ThresParam2);
	iThresParam1 = ThresParam1;
	iThresParam2 = ThresParam2;
	cv::createTrackbar("ThresParam1", "in", &iThresParam1, 13, &TrackerNode::cvTrackBarEventsStatic, this);
	cv::createTrackbar("ThresParam2", "in", &iThresParam2, 13, &TrackerNode::cvTrackBarEventsStatic, this);
	//TrackerNode::cvTrackBarEvents(0,0);
	
	//select the camera using for get image
	//get the first image
/*	if(use_camera)
		openWithVideo0();   
	else
		openWithFlyCap();
	*/
//remember to take the first image for init the camera parameters

	cout<<"FIRST i try to take a image from camera"<<endl;
	Image = FCapture.getImageOpencv();
	cout<<"FIRST return to take a image for processing"<<endl;

	cout<<"IMAGEdata_row187Trackernode : "<<Image.size()<<endl;

	if(Image.data != NULL)
	{	
		cout<<Image.size()<<std::endl;
		//cv::imshow("ScreenFirstImage",Image);
		//int key = cv::waitKey(10); 
		CamParam.resize(Image.size()); 	
	}

	return true;
}


bool TrackerNode::stopTrack(object_detection::stop::Request &req, object_detection::stop::Response &res)
{
	action = false;
	
	//release the stream video
/*	if(use_camera)	vreader.release();
	else		      FCapture.closeVideo();
*/
	destroyAllWindows();//opencv window
	return true;
}

///**************************************************************************+


//************************************************************
//		THREAD 
//************************************************************

void TrackerNode::startStreamThread()
{
	FCapture.init();//open camera and start the thread
	//FCapture.startThread();/
}

///***********************************************************



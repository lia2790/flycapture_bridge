#include "FlyCap.h"

using namespace std;

void FlyCap::init()
{
	cout<<"i'm in init in flycap"<<endl;

	error = camera->Connect(0);//open the stream-video

	if ( error != PGRERROR_OK )
   {    std::cout << "Failed to connect to camera" << std::endl; return;  }  
    
		// Get the camera info and print it out
   error = camera->GetCameraInfo( &camInfo );
   if ( error != PGRERROR_OK )
   {    std::cout << "Failed to get camera info from camera" << std::endl;  return;  } 
  
    std::cout << camInfo.vendorName << " "
    		  << camInfo.modelName << " " 
    		  << camInfo.serialNumber << std::endl;
	
/*
	VideoMode *pVideo;
	FrameRate *pFrame;
	camera->GetVideoModeAndFrameRate(pVideo,pFrame);

	cout<<"VideoMode : "<<*pVideo<<" FrameRate : "<<*pFrame<<endl;
	*/

	error = camera->StartCapture();


   if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
   {    std::cout << "Bandwidth exceeded" << std::endl; return;	}
   else if ( error != PGRERROR_OK )
	{ 	std::cout << "Failed to start image capture" << std::endl; return; 	} 

	cout<<"pre starting thread"<<endl;

	TstreamVideo = std::thread(&FlyCap::startThread, this);	//parte il thread		
	
	cout<<"thread start"<<endl;

	//flyRaw();//take a first image
	
}


void FlyCap::flyRaw()
{
		{
			std::lock_guard<std::mutex> lock(mutexVideo);
			error = camera->RetrieveBuffer( &rawImage );//grab to image
		}
		if ( error != PGRERROR_OK )
			std::cout << "capture error" << std::endl;
}


void FlyCap::startThread()//funzione associata al thread
{
	cout<<"row61FIleFlyCap"<<endl;

	
	while(camera->IsConnected())
	{cout<<"buuuu"<<endl;		flyRaw(); }

	cout<<"row67fileFlycap"<<endl;
}




Mat FlyCap::getImageOpencv()
{
		// convert to rgb with FlyCap

		int size= rawImage.GetDataSize();
		cout<<"dimensione in byte di rawImage : "<<size<<endl;

	   Image rgbImage;
		{
			std::lock_guard<std::mutex> lock(mutexVideo);
   	   rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
      } 
		
		int ssize = rgbImage.GetDataSize();
		cout<<"dim in byte di rgbImage : "<<ssize<<endl;


		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
		image_for_OPENCV = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

	return image_for_OPENCV;

}

/*
void FlyCap::closeVideo()
{
	error = camera->StopCapture();
    if ( error != PGRERROR_OK )
    {
			// This may fail when the camera was removed, so don't show 
        // an error message
    }  
	
	camera->Disconnect();
}
*/

/*    "OLD VERSION"

void FlyCap::startVideo()
{
	 error = camera->Connect(0);//open the stream-video

    if ( error != PGRERROR_OK )
    {    std::cout << "Failed to connect to camera" << std::endl; return;  }  
    


    // Get the camera info and print it out
    error = camera->GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {    std::cout << "Failed to get camera info from camera" << std::endl;  return;  } 
  
    std::cout << camInfo.vendorName << " "
    		  << camInfo.modelName << " " 
    		  << camInfo.serialNumber << std::endl;
	
/*
	VideoMode *pVideo;
	FrameRate *pFrame;
	camera->GetVideoModeAndFrameRate(pVideo,pFrame);

	cout<<"VideoMode : "<<*pVideo<<" FrameRate : "<<*pFrame<<endl;
	

	error = camera->StartCapture();


    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {    std::cout << "Bandwidth exceeded" << std::endl; return;	}
    else if ( error != PGRERROR_OK )
	 { 	std::cout << "Failed to start image capture" << std::endl; return; 	} 
	

	


	//camera->SetVideoModeAndFrameRate(VIDEOMODE_640x480Y8,FRAMERATE_60); //don't work

}


// function calling in the loop (retrieve and convert in Mat for OpenCV in RGB format)

void FlyCap::takeImageFromVideo()
{
		Image rawImage;
		error = camera->RetrieveBuffer( &rawImage );//grab to image
		if ( error != PGRERROR_OK )
			std::cout << "capture error" << std::endl;
		
		// convert to rgb with FlyCap
	   Image rgbImage;
      rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
       
		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
		image_for_OPENCV = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

	//cv::imshow("image", image_for_OPENCV);
	//char key = cv::waitKey(100); //for test and work correctly, the image is correct
}

void FlyCap::closeVideo()
{
	error = camera->StopCapture();
    if ( error != PGRERROR_OK )
    {
			// This may fail when the camera was removed, so don't show 
        // an error message
    }  
	
	camera->Disconnect();
}
 */


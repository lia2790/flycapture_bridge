#include <ros/ros.h>
#include <image_transport/image_transport.h> // includes everything we need to publish and subscribe to images.

//These headers will allow us to display images using OpenCV's simple GUI capabilities.
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using namespace std;





void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
	cout<<"8"<<endl;


	//per vedere solo se al ricevitore mi arriva l 'immagine corretta
    try
    {
    	 cv::imshow("view", cv_bridge::toCvShare(image_msg, "rgb8")->image);
    	 cv::waitKey(30);

		 cout<<"9"<<endl;
    }
    catch (cv_bridge::Exception& e)
    {
       ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
    }

	cout<<"10"<<endl;

//------------ALVAR TRACKING MARKER---->copiato uguale da github

}
	

int main(int argc, char **argv)
{
	cout<<"0"<<endl;

	//inizializzazione nodo ros
   ros::init(argc, argv, "ar_track_alvar");
	ros::NodeHandle n;

	cout<<"1"<<endl;

	//for only view
	cv::namedWindow("view");
	cv::startWindowThread();



	cout<<"2"<<endl;

//	marker_detector.SetMarkerSize(marker_size);

	cout<<"2,1"<<endl;

//	cam = new Camera(n, cam_info_topic); //cam_info_topic è il nome del topic (stringa)
	//dal quale prelevare le info necessarie per inizializzare la camera coi suoi valori intrinseci 
	//non è necessario fare subscriber perchè c'è già dentro alla classe camera, 
	//quindi tu passi semplicemente la stringa ed è tutto pronto
	//basta che sul topic dal quale prelevare il messaggio ci sia la corretta informazione
	//sul buffer viaggia un pacchetto del tipo (messaggio)
	//sensor_msgs::CameraInfo
	//gestione del topic ---> ROS IMAGE TRANSPORT
	//in CAMERA-H
	//95   std::string cameraInfoTopic_;
	//96   sensor_msgs::CameraInfo cam_info_;
	//97   void camInfoCallback (const sensor_msgs::CameraInfoConstPtr &);
	//callback del subscriber


	cout<<"2,2"<<endl;





/*
	//strutture necessarie per il nodo ROSALVAR
	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	arMarkerPub_ = n.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 0);
	rvizMarkerPub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
*/
	cout<<"3"<<endl;	

	



	//prelevare dal buffer l'immagine <-> la matrice 
/*	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe(cam_image_topic, 1, imageCallback);
*/
	


NodeTrack NT("ar_track_alvar");

	
	cout<<"5"<<endl;

	//??for only show 
	//ros::spin();//fa un while dentro
	cv::destroyWindow("view");

	cout<<"6"<<endl;

	
	ros::Rate rate(100);



	while(ros::ok())
	{
		cout<<"7"<<endl;
		ros::spinOnce();//chiede "al ros" cosa c'è da fare
		rate.sleep();
	}
}





//----------> qui mi serve una funzione che mi tiri fuori 
				// dal pacchetto   ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
				// il pacchetto    sensor_msgs::geometry_msgs
				//dove quest'ultimo è quello corretto da far viaggiare sul buffer in broadcast


		//sendTransform(const geometry_msgs::TransformStamped & transform);



		//stampo la posa come broadcaster sul topic in uscita
		//stampo un geometry message_msgs



  /** \brief Send a StampedTransform 
   * The stamped data structure includes frame_id, and time, and parent_id already.  */


// -----------------------> void sendTransform(const StampedTransform & transform);

  /** \brief Send a vector of StampedTransforms 
   * The stamped data structure includes frame_id, and time, and parent_id already.  */


//  --------------------->   void sendTransform(const std::vector<StampedTransform> & transforms);

  /** \brief Send a TransformStamped message
   * The stamped data structure includes frame_id, and time, and parent_id already.  */


// -------------------------> void sendTransform(const geometry_msgs::TransformStamped & transform);

  /** \brief Send a vector of TransformStamped messages
   * The stamped data structure includes frame_id, and time, and parent_id already.  */




 // ---------------------->   void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);*/
	

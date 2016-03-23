
#include <std_msgs/Bool.h>
#include <ar_track_alvar/CvTestbed.h>
#include <ar_track_alvar/MarkerDetector.h>
//#include <ar_track_alvar/Shared.h>



#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_track_alvar/ParamsConfig.h>



/*
This is the callback function that will get called when a new image has arrived on the "camera/image" topic. 
Although the image may have been sent in some arbitrary transport-specific message type, 
notice that the callback need only handle the normal sensor_msgs/Image type. 
All image encoding/decoding is handled automagically for you.

The body of the callback. We convert the ROS image message to an OpenCV image with BGR pixel encoding, 
'then show it in a display window.
*/


using namespace alvar;
using namespace std;
using namespace cv;



class NodeTrack
{
	private: 
				ros::NodeHandle n_;

						


			//elementi nodo ros
			ros::Publisher arMarkerPub_;
			ros::Publisher rvizMarkerPub_;

			

	//prelevare dal buffer l'immagine <-> la matrice 
	image_transport::ImageTransport it;
	image_transport::Subscriber sub; //= it.subscribe(cam_image_topic, 1, imageCallback);

	

			visualization_msgs::Marker rvizMarker_;


			//strutture necessarie per detection
			Camera *cam; 													//per sys di riferimento e matrice intrinseca
			cv_bridge::CvImagePtr cv_ptr_; 							//immagine<->matrice sotto OPENCV
			MarkerDetector<MarkerData> marker_detector; 			//classe dei marker da individuare
			ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;  //posa stimata finale
			//per essere immesso in uscita ar_track_alvar_msgs devo convertirlo in geometry_msgs



			//tf::TransformListener *tf_listener; 		//serve per far dialogare sistemi di riferimento diversi
			tf::TransformBroadcaster *tf_broadcaster; //è una classe che pubblica le trasformate tra sys di riferimento
			tf::TransformListener *tf_listener;			//segue i "movimenti dinamici" le trasformazioni
			double marker_size;
			double max_new_marker_error;
			double max_track_error;


			//topic di interesse
			std::string cam_image_topic; //take from publisher the image
			std::string cam_info_topic;  //take from publisher the matrix
			std::string output_frame;    //-------> poteva essere diverso da quello dell'obiettivo 
			//ma ovviamente per comodità lo si sceglie uguale
			


			void imageCallback(const sensor_msgs::ImageConstPtr& image_msg);

	public:

			NodeTrack();
			~NodeTrack(){};


};




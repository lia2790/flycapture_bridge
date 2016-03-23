#include "alvar_track.h"


NodeTrack::NodeTrack(std::string name_space)
{
	n_ = ros::NodeHandle(name_space);

	it=(n_);

	sub = it.subscribe(cam_image_topic, 1, &NodeTrack::imageCallback, this);



	marker_size = 4.45;
	max_new_marker_error = 0;
	max_track_error = 0;


	cam_image_topic = "camera/image"; 
	cam_info_topic = "camera/camera_info";
	output_frame = "G_opt_camera_info";


}

void NodeTrack::init()
{
	
	

	marker_detector.SetMarkerSize(marker_size);

	cam = new Camera(n, cam_info_topic); //cam_info_topic è il nome del topic (stringa)
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


	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	arMarkerPub_ = n.advertise<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 0);
	rvizMarkerPub_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);


	
	//prelevare dal buffer l'immagine <-> la matrice 
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe(cam_image_topic, 1, imageCallback);



}




void NodeTrack::imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
{

	 //If we've already gotten the cam info, then go ahead
	if(cam->getCamInfo_)
	{
		cout<<"11"<<endl;

		try
		{
			tf::StampedTransform CamToOutput;//ascolta da tf

			cout<<"12"<<endl;

    		try
			{
				cout<<"13"<<endl;			
				
				tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(1.0));
				tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, CamToOutput);

				cout<<"14"<<endl;
   		}
    		catch(tf::TransformException ex)
			{	ROS_ERROR("%s",ex.what()); }

			//Convert the image
         cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

			cout<<"15"<<endl;
      
			IplImage ipl_image = cv_ptr_->image;
         // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
         // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
         // do this conversion here -jbinney

			cout<<"16"<<endl;
        
			//Get the estimated pose of the main markers by using all the markers in each bundle
         marker_detector.Detect(&ipl_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);
         arPoseMarkers_.markers.clear();

			cout<<"17"<<endl;

			for (size_t i=0; i<marker_detector.markers->size(); i++) 
			{
				//Get the pose relative to the camera
        		int id = (*(marker_detector.markers))[i].GetId(); 
				Pose p = (*(marker_detector.markers))[i].pose;
				double px = p.translation[0]/100.0;
				double py = p.translation[1]/100.0;
				double pz = p.translation[2]/100.0;
				double qx = p.quaternion[1];
				double qy = p.quaternion[2];
				double qz = p.quaternion[3];
				double qw = p.quaternion[0];

            tf::Quaternion rotation(qx,qy,qz,qw);
            tf::Vector3 origin(px,py,pz);
            tf::Transform t(rotation, origin);
            tf::Vector3 markerOrigin (0, 0, 0);
            tf::Transform m(tf::Quaternion::getIdentity(), markerOrigin);
            tf::Transform markerPose = t * m; // marker pose in the camera frame

				tf::Vector3 z_axis_cam = tf::Transform(rotation, tf::Vector3(0,0,0))*tf::Vector3(0, 0, 1);
				//ROS_INFO("%02i Z in cam frame: %f %f %f",id, z_axis_cam.x(), z_axis_cam.y(), z_axis_cam.z());
				/// as we can't see through markers, this one is false positive detection
				
				if (z_axis_cam.z() > 0){ continue; }

				//Publish the transform from the camera to the marker		
				std::string markerFrame = "ar_marker_";
				std::stringstream out;
				out<<id;
				std::string id_string = out.str();
				markerFrame += id_string;
				tf::StampedTransform camToMarker(t, image_msg->header.stamp, image_msg->header.frame_id, markerFrame.c_str());
    			tf_broadcaster->sendTransform(camToMarker);//pubblicare la trasf su TF(è un pacchetto con un topic)
	
			
				//Create the rviz visualization messages
				tf::poseTFToMsg(markerPose, rvizMarker_.pose);
				rvizMarker_.header.frame_id = image_msg->header.frame_id;
				rvizMarker_.header.stamp = image_msg->header.stamp;
				rvizMarker_.id = id;

				rvizMarker_.scale.x = 1.0 * marker_size/100.0;
				rvizMarker_.scale.y = 1.0 * marker_size/100.0;
				rvizMarker_.scale.z = 0.2 * marker_size/100.0;
				rvizMarker_.ns = "basic_shapes";
				rvizMarker_.type = visualization_msgs::Marker::CUBE;
				rvizMarker_.action = visualization_msgs::Marker::ADD;
				switch (id)
				{
				  case 0:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 1.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 1:
				    rvizMarker_.color.r = 1.0f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 0.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 2:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 1.0f;
				    rvizMarker_.color.b = 0.0f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 3:
				    rvizMarker_.color.r = 0.0f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				  case 4:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.5f;
				    rvizMarker_.color.b = 0.0;
				    rvizMarker_.color.a = 1.0;
				    break;
				  default:
				    rvizMarker_.color.r = 0.5f;
				    rvizMarker_.color.g = 0.0f;
				    rvizMarker_.color.b = 0.5f;
				    rvizMarker_.color.a = 1.0;
				    break;
				}
				rvizMarker_.lifetime = ros::Duration(1.0);
				rvizMarkerPub_.publish(rvizMarker_);

				//Get the pose of the tag in the camera frame, then the output frame (usually torso)				
				tf::Transform tagPoseOutput = CamToOutput * markerPose;

				//Create the pose marker messages
				ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
				tf::poseTFToMsg(tagPoseOutput, ar_pose_marker.pose.pose);
      		ar_pose_marker.header.frame_id = output_frame;
			   ar_pose_marker.header.stamp = image_msg->header.stamp;
			   ar_pose_marker.id = id;
			   arPoseMarkers_.markers.push_back(ar_pose_marker);	
			}


			//qui dentro ho la posa stimata
			//ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
			arMarkerPub_.publish(arPoseMarkers_);
			//srà una classe all'interno della quale dovrò andare a prelevare
			//taluna posa e pubblicarla in broadcast
			//sul topica di uscita
			//come un sensor_msgs::geometry_msgs

		}
       catch(cv_bridge::Exception& e)
		{
      		ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str());
    	}
	}//end_if



}


  















    
    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <opencv2/highgui/highgui.hpp>
    #include <cv_bridge/cv_bridge.h>
    #include <sensor_msgs/image_encodings.h> 
    #include <image_geometry/pinhole_camera_model.h>   
   #include <sensor_msgs/distortion_models.h>   
   #include <message_filters/sync_policies/approximate_time.h>

   image_transport::Publisher pub ; 
  
 void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {
	cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
	//Undistort image
       sensor_msgs::CameraInfoPtr camera = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo());
       camera->D.push_back(0.0);
       camera->D.push_back(0.0);
       camera->D.push_back(0.0);
       camera->D.push_back(0.0);
       camera->D.push_back(0.0);

       camera->K[0] = 525.0; 
       camera->K[1] = 0.000000;
       camera->K[2] = 319.5;
       camera->K[3] = 0.000000;
       camera->K[4] = 516.5;
       camera->K[5] = 239.5;
       camera->K[6] = 0.000000;
       camera->K[7] = 0.000000;
       camera->K[8] = 1.000000;

      camera->height = 480;
      camera->width  = 640;
    
      camera->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

      camera->P[0] = 1.0; 	
      camera->P[1] = 0.000000;	
      camera->P[2] = 0.00;	
      camera->P[3] = 0.000000;	
      camera->P[4] = 1.0;	
      camera->P[5] = 0.0;	
      camera->P[6] = 0.0;	
      camera->P[7] = 0.000000;	
      camera->P[8] = 1.0;	
      camera->P[9] = 0.000000;	
      camera->P[10] = 0.000000;	
      camera->P[11] = 0.000000;

     camera->R[0]= 1.000000 ;    	
     camera->R[1]= 0.000000;    	
     camera->R[2]= 0.000000;    	
     camera->R[3]= 0.000000;    	
     camera->R[4]= 1.000000;    	
     camera->R[5]= 0.000000;    	
     camera->R[6]= 0.000000;    	
     camera->R[7]= 0.000000;    	
     camera->R[8]= 1.000000;    	
     
     image_geometry::PinholeCameraModel *cam_model = new image_geometry::PinholeCameraModel();
     cam_model->fromCameraInfo(camera);
    
     cv::Mat rectified_img =  cv::Mat(img.rows,img.cols,CV_8UC3);	
     cam_model->rectifyImage(img,rectified_img);
    //ROS_INFO("%f\n", camera_info->K[0]);

    cv::imshow("distorted_image", img);
    cv::imshow("undistorted_image", rectified_img);
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rectified_img).toImageMsg();
    pub.publish(msg);

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
     cv::namedWindow("distorted_image",cv::WINDOW_NORMAL);
     cv::startWindowThread();

     cv::namedWindow("undistorted_image",cv::WINDOW_NORMAL);
     cv::startWindowThread();

     image_transport::ImageTransport it(nh);

     image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);
   


     pub = it.advertise("/undistorted_image_topic", 1);

     ros::spin();
     cv::destroyWindow("distorted_image");
   }

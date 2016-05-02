#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "exercise1/filter_state.h"
#include "tum_ardrone/filter_state.h"
#include <sophus/se3.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "photo_goal_nav.hpp"
#include "Transformation.hpp"
#include <std_msgs/String.h>
#include "TooN/TooN.h"
#include "TooN/so3.h"
#include "TooN/se3.h"


using namespace cv;
using namespace std;

#define PI 3.14159265

Mat image2;
bool keyboardEventReceived = false;
int frame_no = 1;
bool image_2_reached=true ;

// float focal_length_x=0.771557;
// float focal_length_y=1.368560;
//float c_x=0.552779;
//float c_y=0.444056;
//float scew=1.156010;

float focal_length_x = 579.845976;
float focal_length_y = 578.585421;
float c_x = 349.370674;
float c_y = 167.103928;
float scew = 0.0;
vector<float> distCoeff;
Mat K1 = Mat(3, 3, CV_32F, cvScalar(0.));

ros::Publisher pub_com;
ros::Publisher pub_clear_com;

pthread_mutex_t logControl_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t image2lock = PTHREAD_MUTEX_INITIALIZER;

tum_ardrone::filter_stateConstPtr quad_poseConstPtr;

IplImage*  status_image ;
int status_image_width=2*640;
int status_image_height=2*360+50;
bool debug_mode=false;

struct quad_considered_pose {

	float x;
	float y;
	float z;
	float yaw;
	float pitch;
	float roll;
} ;

struct command_log{

	float x;
	float y;
	float z;
	float yaw;
};

command_log current_momentum;
command_log position_T_minus2,position_Tminus1,positionT;
quad_considered_pose quad_considered_pose_obj;

int log_count;

bool target_pose_achieved ;
double target_pose_achieved_time ;

struct Xgreater {
	bool operator()(const DMatch& matchx, const DMatch& matchr) const {
		return matchx.distance < matchr.distance;

	}
};

void set_target_achieved_status();
void update_matched_image_in_status_figure(Mat matched_image);
void update_target_image_in_status_figure(Mat image1);
void update_current_image_in_status_figure(Mat image1);
void update_command_sent_status(string status);
void reset_momentum();

int  run_homography_and_validate(vector<DMatch> matches,vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, Mat image1,Mat image2);

void send_clear_command(){

	std_msgs::String empty_msg;
	pub_clear_com.publish(empty_msg);

}

int run_eight_point_algo_on_matches(vector<DMatch> matches,vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, Mat image1, Mat image2);

int run_eight_point_algo_on_matches1(vector<DMatch> matches,
		vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, Mat image1,
		Mat image2);

void drawEpilines(Mat fMatrix, std::vector<Point2f> objPoints,
		std::vector<Point2f> scenePoints, Mat image1, Mat image2) {

}

bool isSameDirection(float t0,float t1,float t2)
{
	if(((t0>t1) && (t1>t2)) || ((t1>t0) && (t2>t1))){
		return true;

	}
	return false;

}

float calculate_step_size(float current,float target){
	float step=0.2;
//	if(current - target  > max_step){
//		step = - max_step;
//	}else if(target - current > max_step){
//		step = max_step;
//	}
	if (current > target)
		return -step;

	return step;
}

float calculate_step_size1(float current,float target){
	return target-current;
}


void log_go_to_command(float target_x,float target_y,float target_z,float target_yaw){

position_T_minus2.x = position_Tminus1.x;
position_T_minus2.y = position_Tminus1.y;
position_T_minus2.z = position_Tminus1.z;
position_T_minus2.yaw = position_Tminus1.yaw;

position_Tminus1.x = positionT.x;
position_Tminus1.y = positionT.y;
position_Tminus1.z = positionT.z;
position_Tminus1.yaw = positionT.yaw;

 positionT.x = target_x;
 positionT.y = target_y;
 positionT.z = target_z;
 positionT.yaw = target_yaw;

 log_count++;
}

command_log calculate_go_to_command(quad_considered_pose considered_pose,float target_x,float target_y,float target_z,float target_yaw){
	float step_x,step_y,step_z;

	step_x = calculate_step_size(considered_pose.x,target_x);
	step_y = calculate_step_size(considered_pose.y, target_y);
	step_z = calculate_step_size(considered_pose.z, target_z);

	command_log com_goto;
	com_goto.x = considered_pose.x + current_momentum.x*step_x;
	com_goto.y = considered_pose.y + current_momentum.y*step_y;
	com_goto.z = considered_pose.z + current_momentum.z*step_z;
	com_goto.yaw = 0;

	return com_goto;
}

command_log calculate_go_to_command2(quad_considered_pose considered_pose,float target_x,float target_y,float target_z,float target_yaw){
	float step_x,step_y,step_z,step_yaw;

	step_x = calculate_step_size1(considered_pose.x,target_x);
	step_y = calculate_step_size1(considered_pose.y, target_y);
	step_z = calculate_step_size1(considered_pose.z, target_z);
	step_yaw = calculate_step_size1(considered_pose.yaw, target_yaw);

	float factor=0.5;
	command_log com_goto;
	com_goto.x = considered_pose.x + factor * step_x;
	com_goto.y = considered_pose.y + factor * step_y;
	com_goto.z = considered_pose.z + factor * step_z;
	com_goto.yaw = considered_pose.yaw +factor * step_yaw;

	return com_goto;
}


void update_momentum(){

	if(log_count <3){
		return;
	}
	float dec_factor =0.5;
	if(!isSameDirection(position_T_minus2.x,position_T_minus2.x,positionT.x)){
		current_momentum.x= current_momentum.x* dec_factor;
	}
	if(!isSameDirection(position_T_minus2.y,position_T_minus2.y,positionT.y)){
		current_momentum.y= current_momentum.y* dec_factor;
	}
	if(!isSameDirection(position_T_minus2.z,position_T_minus2.z,positionT.z)){
		current_momentum.z= current_momentum.z* dec_factor;
	}

}


void setRefCommand() {
	std_msgs::String msg;

	char buf[50] = "c setReference 0 0 0 0";
	//sprintf(buf,"c goto %f %f %f %f");
	msg.data = buf;
	pub_com.publish(msg);

}

void gotoCommand(float x, float y, float z, float yaw) {
	cout << "sending goto: " << x << " " << y << " " << z << endl;

	std_msgs::String msg;
	setRefCommand();
	char buf[50];
	sprintf(buf, "c goto %f %f %f %f", x, y, z, yaw);
	msg.data = buf;

	//cout << "going to publish"  << endl ;
	send_clear_command();
	pub_com.publish(msg);


	target_pose_achieved=false ;
	string status(buf);
	update_command_sent_status(status.substr(6));

	//cout << "published"  << endl ;
}

void moveByRelCommand(float x, float y, float z, float yaw) {
	cout << "sending goto: " << x << " " << y << " " << z << endl;

	std_msgs::String msg;

	char buf[50];
	sprintf(buf, "c moveByRel %f %f %f %f", x, y, z, yaw);
	msg.data = buf;
	pub_com.publish(msg);

}


void rod2rpy(Mat trans, float* roll, float* pitch, float* yaw) {
	//Do we need this inverse
	Mat mat = trans.inv();

	*roll = atan2(-mat.at<float>(0, 2),
			sqrt(
					mat.at<float>(0, 0) * mat.at<float>(0, 0)
							+ mat.at<float>(0, 1) * mat.at<float>(0, 1)));

	*yaw = atan2(mat.at<float>(0, 1) / cos(*roll),
			mat.at<float>(0, 0) / cos(*roll));

	*pitch = atan2(mat.at<float>(1, 2) / cos(*roll),
			mat.at<float>(2, 2) / cos(*roll));

	*pitch *= 180 / 3.14159265;
	*roll *= 180 / 3.14159265;
	*yaw *= -180 / 3.14159265;

	while (*pitch > 180)
		*pitch -= 360;
	while (*pitch < -180)
		*pitch += 360;
	while (*roll > 180)
		*roll -= 360;
	while (*roll < -180)
		*roll += 360;
	while (*yaw > 180)
		*yaw -= 360;
	while (*yaw < -180)
		*yaw += 360;
}

double getYawAngleFromRotationMatrix(Mat rotationMatrix) {

	Eigen::Matrix3f c;

	cv2eigen(rotationMatrix, c);

	//	     cout << "\nEigen Rotation Matrix" << rotationMatrix << endl ;

	Eigen::Quaternionf orientation(c);

	double x = orientation.x();
	double y = orientation.y();
	double z = orientation.z();
	double w = orientation.w();

	//cout << "X:" << x << "Y" << y << "Z::" << z << "W::" << w << std::endl;

	double current_yaw_angle = atan2(2 * x * y + 2 * w * z,
			w * w + x * x - y * y - z * z);

	// double roll=(control_force[0]*sin(current_yaw_angle)- control_force[1]*cos(current_yaw_angle))/g;
	// double pitch=(control_force[0]*cos(current_yaw_angle) + control_force[1]*sin(current_yaw_angle))/g
	return current_yaw_angle;
}

void getFramesFromVideo(Mat image) {
	cout << "Frame No" << frame_no << "\n" << endl;
	cout << "Frame Size:: \n Rows" << image.rows << "\nCols" << image.cols
			<< "\nSize" << image.size() << endl;

	cout << "Saving image" << endl;

	String result = "video_seq_images/frames_seq_"
			+ boost::lexical_cast<std::string>(frame_no) + ".png";
	try {
		imwrite(result, image);
	} catch (runtime_error& ex) {
		fprintf(stderr, "Exception converting image to PNG format: %s\n",
				ex.what());

	}
	frame_no = frame_no + 1;

}





void ratio_test_for_matches(vector<DMatch> &passed_matches, vector<vector<DMatch> > k_matches,Mat image1, vector<KeyPoint> keyPoints1, Mat image2, vector<KeyPoint> keyPoints2){

	for(int i=0; i < k_matches.size();i++){
		vector<DMatch> matches_for_this_point=k_matches[i];
		DMatch first_match=matches_for_this_point[0];		

		if(matches_for_this_point.size() == 1){
			passed_matches.push_back(first_match);	
//			cout << "For point size of matchs is" << matches_for_this_point.size()<< endl ;
		}else {
			DMatch second_match=matches_for_this_point[1];
	   		 if (first_match.distance < 0.75*second_match.distance){
        			passed_matches.push_back(first_match);
			}
//			else {
//			cout << "Not using this point as first distance" << first_match.distance << "second distance " << second_match.distance << endl ; 
//			}
		}
	}
 	Mat img_matches;
        drawMatches(image1, keyPoints1, image2, keyPoints2, passed_matches,img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	//imshow("matches_after_ratio_test", img_matches);

	cout << "No of matches received " << k_matches.size() << " for ratio test. No of matches that passed ratio test " << passed_matches.size() << endl ;

}



void getGoodMatches(vector<DMatch> &good_matches, vector<DMatch> matches,
		Mat image1, vector<KeyPoint> keyPoints1, Mat image2,
		vector<KeyPoint> keyPoints2) {

	double max_dist = 0;
	double min_dist = 100;

	//This is always less than descriptor 1
	int matches_size = matches.size();

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < matches_size; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	cout << "Initial good_matches size :" << good_matches.size() << endl;
	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.

	for (int i = 0; i < matches_size; i++) {
		//if (matches[i].distance <= max(2 * min_dist, 0.02)) {
		if (matches[i].distance <= max(2 * min_dist, 0.02)) {
			good_matches.push_back(matches[i]);
		}
	}

	Mat img_matches;
	drawMatches(image1, keyPoints1, image2, keyPoints2, good_matches,
			img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(),
			DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	cout << "Checking size of images: image1 size" << image1.size() << "image2 size" << image2.size() <<  "img_matches size" << img_matches.size() << endl ;

	//-- Show good matches
	update_matched_image_in_status_figure(img_matches);
	//imshow("good_matches", img_matches);

	//		for (int i = 0; i < (int) good_matches.size(); i++) {
	//			printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n",
	//					i, good_matches[i].queryIdx, good_matches[i].trainIdx);
	//		}

	cout << "good_matches size :" << good_matches.size() << endl;

	return;

	//		// Sort them in the order of their distance.
	//		 sort(matches.begin(), matches.end(),Xgreater());
	//		 matches.resize(2);
	////		 cout << "Ist match distance" << matches.at(0).distance  << endl ;
	////		 cout << "2st match distance" << matches.at(1).distance  << endl ;
	////		 cout << "3rd match distance" << matches.at(2).distance  << endl ;
	////		 cout << "4th match distance" << matches.at(3).distance  << endl ;
	////		 cout << "5th match distance" << matches.at(4).distance  << endl ;
	//		 cout << "No of matches found " << matches.size() << endl ;
	//		 Mat img_matches;
	//				 if(matches.size() > 0){
	//				 	 drawMatches(image1, keyPoints1, image2, keyPoints2,
	//						matches, img_matches,Scalar::all(-1), Scalar::all(-1),
	//			               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	//
	//				//-- Show detected matches
	//				//	 cv::imshow("video_stream", image1);
	//				 	 cout << "drawing matches" << endl ;
	//				}
	//				imshow("matches", img_matches);

	//

}



void findAllMatchesImage1To2(vector<vector<DMatch> >& k_matches, Mat image1,
		vector<KeyPoint> &keyPoints1, Mat &descriptors1, Mat image2,
		vector<KeyPoint> &keyPoints2, Mat &descriptors2) {

	//Fast is  not scale invariant
	//		cv::FeatureDetector * detector = new cv::OrbFeatureDetector();
	//		//ORB descriptor is extension of BRIEF //not scale invariant
	//		cv::DescriptorExtractor * extractor = new cv::OrbDescriptorExtractor();


	Ptr< xfeatures2d::SIFT > detector_extractor = xfeatures2d::SIFT::create(0, 3, 0.07, 10, 1.6);

	if (detector_extractor == NULL) {
		cout << "detector not found " << endl;
		return;
	}

	detector_extractor->detect(image1, keyPoints1);
	detector_extractor->detect(image2, keyPoints2);

	//		cout << "Before No of keypoints in image1" << keyPoints1.size() << endl ;
	//		cout << "Before No of keypoints in image2" << keyPoints2.size() << endl ;

	detector_extractor->compute(image1, keyPoints1, descriptors1);

	detector_extractor->compute(image2, keyPoints2, descriptors2);

	if (descriptors1.type() != CV_32F) {
		descriptors1.convertTo(descriptors1, CV_32F);
	}

	if (descriptors2.type() != CV_32F) {
		descriptors2.convertTo(descriptors2, CV_32F);
	}

	cout << "\nNo of keypoints in image1 :: " << keyPoints1.size() << endl;
	cout << "\nNo of keypoints in image2 :: " << keyPoints2.size() << endl;
	cout << "\nDescriptor1 size" << descriptors1.size() << endl;
	cout << "\nDescriptor2 size" << descriptors2.size() << endl;

	BFMatcher matcher(NORM_L2, false);	//crosCheck=true

	//matcher.match(descriptors1, descriptors2, matches);
	int k=2 ;
	matcher.knnMatch(descriptors1, descriptors2, k_matches,k);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	//		FlannBasedMatcher matcher;
	//		std::vector<DMatch> matches;
	//
	//		matcher.match(descriptors1, descriptors2, matches);

	cout << "Desc rows" << descriptors1.rows << endl;
	//cout << "Matches Size Inside" << matches.size() << endl;

	//-- Draw ALL matches
	//Mat img_matches;
	//drawMatchesknn(image1, keyPoints1, image2, keyPoints2, k_matches, img_matches,Scalar::all(-1), Scalar::all(-1), vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//-- Show detected matches
	//imshow("matches", img_matches);

}




int calculate(Mat image1) {

	//getFramesFromVideo(image2);



	if (!image2.data)                             // Check for invalid input
	{
		cout << "Could not open or find the image2" << std::endl;
		return 0;
	}

	Mat gray1;

	vector<KeyPoint> keyPoints1;
	vector<KeyPoint> keyPoints2;
	Mat descriptors1;
	Mat descriptors2;
	std::vector<vector<DMatch> > k_matches;

	findAllMatchesImage1To2(k_matches, image1, keyPoints1, descriptors1, image2, keyPoints2, descriptors2);

	cout << "-- Initially Matches Size: " << k_matches.size() << endl;
	

	std::vector<DMatch> passed_matches;
	ratio_test_for_matches(passed_matches, k_matches, image1, keyPoints1, image2, keyPoints2);

	std::vector<DMatch> good_matches;

	getGoodMatches(good_matches, passed_matches, image1, keyPoints1, image2,keyPoints2);


	//8 point algorithm

	std::vector<DMatch> matches; //contains first match
        for(int i=0; i < k_matches.size();i++){
                vector <DMatch> m =k_matches[i];
                matches.push_back(m[0]);
        }

        //Either use matches or passed_matches or good_matches

	int status = 0;
	int use_homography=true;

	if(use_homography){
		if (good_matches.size() >= 4) {
				cout << "Going to run homography" << endl;
				cout << "No of points" << good_matches.size() << endl ;
				status=run_homography_and_validate(good_matches, keyPoints1,keyPoints2, image1, image2);
			} else {
				cout << "not enough points for homography" << endl;
			}
	}else {
		if (passed_matches.size() >= 8) {
				cout << "Going to run 8 point algo" << endl;
				cout << "No of points" << good_matches.size() << endl ;
				status = run_eight_point_algo_on_matches1(passed_matches, keyPoints1,keyPoints2, image1, image2);
				//status = run_eight_point_algo_on_matches(good_matches, keyPoints1,keyPoints2, image1, image2);
		} else {
				cout << "not enough points for 8-point algo " << endl;
			}

	}

	return status;


}

Mat rpy2rod(float roll, float pitch, float yaw) {
	Mat mat(3, 3, CV_32F);

	pitch /= 180 / 3.14159265;
	roll /= 180 / 3.14159265;
	yaw /= -180 / 3.14159265;

	double sa = sin(yaw);	// a is yaw = psi
	double ca = cos(yaw);
	double sb = sin(roll);	// b is roll = phi
	double cb = cos(roll);
	double sg = sin(pitch);	// g is pitch = theta
	double cg = cos(pitch);

	mat.at<float>(0, 0) = ca * cb;
	mat.at<float>(0, 1) = sa * cb;
	mat.at<float>(0, 2) = -sb;

	mat.at<float>(1, 0) = ca * sb * sg - sa * cg;
	mat.at<float>(1, 1) = sa * sb * sg + ca * cg;
	mat.at<float>(1, 2) = cb * sg;

	mat.at<float>(2, 0) = ca * sb * cg + sa * sg;
	mat.at<float>(2, 1) = sa * sb * cg - ca * sg;
	mat.at<float>(2, 2) = cb * cg;

	//mat = mat.T();
	return mat.inv();
}

void add_rotations() {

	float yaw1, roll1, pitch1, yaw2, roll2, pitch2;

	roll1 = 3;
	pitch1 = 3;
	yaw1 = 3;
	cv::Mat pose_rotation = rpy2rod(roll1, pitch1, yaw1);

	roll2 = 2;
	pitch2 = 7;
	yaw2 = 10;
	Mat validR = rpy2rod(roll2, pitch2, yaw2);

	Mat rotaion_from_world_frame = pose_rotation * validR;

	float yaw, roll, pitch;
	rod2rpy(rotaion_from_world_frame, &roll, &pitch, &yaw);
	cout << " roll" << roll << " pitch" << pitch << " yaw::" << yaw << endl;

}

void transform_cordinates_and_send(Mat validR, Mat validT) {
	//add_rotations();

	float yaw, roll, pitch;
	float X, Y, Z;

	pthread_mutex_lock(&logControl_CS);

	cout << "Pose x:: " << quad_considered_pose_obj.x << endl ;
	X = quad_considered_pose_obj.x;
	Y = quad_considered_pose_obj.y;
	Z = quad_considered_pose_obj.z;
	yaw = quad_considered_pose_obj.yaw;
	roll = quad_considered_pose_obj.roll;
	pitch = quad_considered_pose_obj.pitch;

	pthread_mutex_unlock(&logControl_CS);

	//X=1; Y=0 ; Z=0 ;
	//rod2rpy(Mat::eye(3,3,CV_32FC1),&roll,&pitch,&yaw);



	cv::Mat pose_rotation = rpy2rod(roll, pitch, yaw);
	//cv::Mat pose_rotation=rpy2rod(0,0,0);

	//cout << "\nPose rotation computed" << pose_rotation << endl;

	//double roll;
	//double pitch;
	//cv::Mat rotation = (cv::Mat_<float>(3, 3) << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1);

	cv::Mat pose_translation = (cv::Mat_<float>(3, 1) << X, Y, Z);

	std::cout << "\npose_rotation:: " << pose_rotation << std::endl;
	std::cout << "\npose_translation:: " << pose_translation << std::endl;

	//Mat demo=pose_rotation*pose_translation;
	//cout << "demo" << demo << endl ;

	//new position of camera w.r.t to current position of camera
	cout << "validR type" <<validR.type() << "validT type" << validT.type() << endl ;
	cv::Mat new_camera_center = -validR.t() * validT;

	cout << "validR type" <<validR.type() << "validT type" << validT.type() << "new camera center type" << new_camera_center.type() << endl ;

	cv::Mat new_camera_center_orientation = validR.t();
	//cv::Mat new_camera_center_orientation = validR;
	new_camera_center_orientation.convertTo(new_camera_center_orientation, CV_32F);

	TooN::SO3<float> new_camera_orientation_toon = toToonSO3(new_camera_center_orientation);
	cout<<"new_camera_orientation_toon = "<<new_camera_orientation_toon<<endl;

	float new_camera_yaw, new_camera_roll, new_camera_pitch;
	rod2rpy(new_camera_center_orientation,&new_camera_roll,&new_camera_pitch,&new_camera_yaw);

	TooN::SE3<float> frontToGlobal =  getCurrentFrontToGlobalTransformation(X,Y,Z,roll,pitch,yaw);
	TooN::SE3<float> new_camera_pose ;

	new_camera_pose.get_translation()[0] = new_camera_center.at<double>(0,0);
	new_camera_pose.get_translation()[1] = new_camera_center.at<double>(1,0);
	new_camera_pose.get_translation()[2] = new_camera_center.at<double>(2,0);
	new_camera_pose.get_rotation() = new_camera_orientation_toon;

	TooN::SE3<float> globalCameraPose = frontToGlobal * new_camera_pose;
	cout<<"front_to_global = "<<frontToGlobal<<endl;
	cout<<"global_camera_pose = "<<globalCameraPose<<endl;
	float global_camera_yaw,global_camera_roll, global_camera_pitch;
	rod2rpyToon(globalCameraPose.get_rotation(),&global_camera_roll,&global_camera_pitch,&global_camera_yaw);
	cout << "\nglobal_camera_roll:: "<<global_camera_roll << " global_camera_pitch:: " << global_camera_pitch<< "global_camera_yaw:: " << global_camera_yaw <<endl;



	//new_camera_center.at<float>(0,0)=0;
	//new_camera_center.at<float>(0,1)=0;
	//new_camera_center.at<float>(0,2)=1;

	cout << "\nnew_camera_center:: " << new_camera_center << endl;
	cout << "\nnew_camera_orientation:: " << new_camera_center_orientation << endl;
	cout << "\nnew_camera_roll:: "<<new_camera_roll << " new_camera_pitch:: " << new_camera_pitch<< "new_camera_yaw:: " << new_camera_yaw <<endl;

	Mat transformed_camera_center = Mat::zeros(3, 1, CV_32F);

	cout << "\ntransformed_camera_center initially:: " << transformed_camera_center << "size of new_camera_center" << new_camera_center.size() << "transformed_camera_center size" << transformed_camera_center.size()<< "new_camera_center type" <<new_camera_center.type() <<  "transformed_camera_center type" << transformed_camera_center.type()<< endl;

	transformed_camera_center.at<float>(0, 0) = new_camera_center.at<double>(0,0);
	transformed_camera_center.at<float>(1, 0) = -new_camera_center.at<double>(2,0);
	transformed_camera_center.at<float>(2, 0) = -new_camera_center.at<double>(1,0);

	//yaw and pitch reversed
	//transformed_camera_center.at<float>(0, 0) = validT.at<double>(0,0);
	//transformed_camera_center.at<float>(1, 0) = validT.at<double>(2,0);
	//transformed_camera_center.at<float>(2, 0) = validT.at<double>(1,0);

	//cv::Mat transformed_camera_center_orientation = rpy2rod(new_camera_roll,new_camera_yaw, new_camera_pitch);
	cv::Mat rotx = Mat::zeros(3,3,CV_32F);

	rotx.at<float>(0,0) =1;
	rotx.at<float>(1,2) =-1;
	rotx.at<float>(2,1) =-1;

	cv::Mat transformed_camera_center_orientation = rotx * new_camera_center_orientation;
	float transformed_camera_yaw,  transformed_camera_roll,  transformed_camera_pitch;
	rod2rpy(transformed_camera_center_orientation,&transformed_camera_roll,&transformed_camera_pitch,&transformed_camera_yaw);


	cout << "\ntransformed_camera_center:: " << transformed_camera_center << endl;
	cout << "\ntransformed_camera_center_orientation:: " << transformed_camera_center_orientation << endl;
	cout << "\ntransformed_camera_roll:: "<<transformed_camera_roll << " transformed_camera_pitch:: " << transformed_camera_pitch<< "transformed_camera_yaw:: " << transformed_camera_yaw <<endl;
	//new_position=RX+T
	cv::Mat final_position = pose_rotation * transformed_camera_center + pose_translation;
	std::cout << "\nfinal_position w.r.t world frame:: " << final_position;

	float final_x_position = final_position.at<float>(0, 0);
	float final_y_position = final_position.at<float>(0, 1);
	float final_z_position = final_position.at<float>(0, 2);

	//new rotation=add the two rotations
	//in what order to multiply??
	Mat final_rot = pose_rotation * transformed_camera_center_orientation;

	cout << "\nfinal rotation w.r.t world frame:: " << final_rot << endl;

	float final_roll, final_pitch, final_yaw;
	rod2rpy(final_rot, &final_roll, &final_pitch, &final_yaw);
	cout << "\nfinal_camera_roll:: "<<final_roll << " final_camera_pitch:: " << final_pitch<< "final_camera_yaw:: " << final_yaw <<endl;
	//Sending current pose yaw
	//final_yaw=yaw ;
	//just for testing..this yaw is not used currently
	double yaw_again = getYawAngleFromRotationMatrix(final_rot);
	final_yaw = yaw + new_camera_roll/2;
	cout << "\nyaw from method1 :: " << final_yaw << "\nyaw from method2:: " << yaw_again << "\nusing method 1" << endl;

	cout << "\nsending command X="<< final_x_position << " Y=" << final_y_position << " Z=" << final_z_position << " Yaw=" << final_yaw << endl;


	final_x_position=globalCameraPose.get_translation()[0];
	final_y_position=globalCameraPose.get_translation()[1];
	final_z_position=globalCameraPose.get_translation()[2];
	final_yaw=global_camera_yaw;

	command_log com_goto = calculate_go_to_command2(quad_considered_pose_obj,final_x_position,final_y_position,final_z_position,final_yaw);

	update_momentum();
    cout<<"Momentum_x : "<<current_momentum.x << endl ;
    cout<<"Momentum_y : "<<current_momentum.y << endl;
    cout<<"Momentum_z : "<<current_momentum.z << endl;

	cout << "\nPose Roll:: " << roll << " Pose Pitch:: " << pitch << " Pose yaw:: " << yaw << endl;
	cout << "\nPose X:: " << X << " Pose Y:: " << Y << " Pose Z:: " << Z << endl;

	//gotoCommand(com_goto.x, com_goto.y, com_goto.z, final_yaw);

	float momentum_threshold=0.01;

	if(current_momentum.x  < momentum_threshold && current_momentum.y < momentum_threshold && current_momentum.z < momentum_threshold ){
		cout << "Target Achieved!!" << endl ;
		keyboardEventReceived=false ;
		image_2_reached=true ;
		set_target_achieved_status();
		reset_momentum();
		return ;
	}

	log_go_to_command(com_goto.x,com_goto.y,com_goto.z,com_goto.yaw);
	gotoCommand(com_goto.x,com_goto.y,com_goto.z,com_goto.yaw);

}

void keyboardCallback(const std_msgs::Empty& toggle_msg) {
	std::cout << "keyboard event received" << std::endl;
	keyboardEventReceived = true;
}

void setConsideredPose() {
	if(quad_poseConstPtr != NULL){
		pthread_mutex_lock(&logControl_CS);
		//Save the most immediate pose to be considered

		//cout << "setting considered pose" << endl ;
		quad_considered_pose_obj.x = quad_poseConstPtr->x;
		quad_considered_pose_obj.y = quad_poseConstPtr->y;
		quad_considered_pose_obj.z = quad_poseConstPtr->z;
		quad_considered_pose_obj.yaw = quad_poseConstPtr->yaw;
		quad_considered_pose_obj.pitch = quad_poseConstPtr->pitch;
		quad_considered_pose_obj.roll = quad_poseConstPtr->roll;

		//cout << "Pose set x=" << quad_considered_pose_obj.x << "from " <<quad_poseConstPtr->x << endl ;

		pthread_mutex_unlock(&logControl_CS);
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	if(debug_mode){
			return  ;
		}

	//cout << "image callback received" << endl ;

	Mat image1 = cv_bridge::toCvShare(msg, "bgr8")->image;
	Mat undistort_image1;
	undistort(image1, undistort_image1, K1, distCoeff);
	image1 = undistort_image1;

	update_current_image_in_status_figure(image1);

	try {
		pthread_mutex_lock(&image2lock);
		if (!image_2_reached && keyboardEventReceived) {
			cout << "going to process current frame " << endl;
			setConsideredPose();

			int status = 0;
			status = calculate(image1);
			//status=1 ; //comment this when u uncomment previous line
			if (status == 1) {
				//cout << "Success" << endl;
				keyboardEventReceived = false;
			} else {
				cout << "failed.trying for next frame" << endl;
				//to be changed later
				keyboardEventReceived = false;
			}

		}		// cv::imshow("video_stream", image2);

	} catch (...) {
		cout << "Failed" << endl;
		keyboardEventReceived = false;
	}
	pthread_mutex_unlock(&image2lock);

}

int run_homography_and_validate(vector<DMatch> matches,vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, Mat image1,Mat image2) {
 	
	std::cout<< "\ngoing to run homography to determine rotation and translation" << std::endl;

        std::vector<Point2f> objPoints;
        std::vector<Point2f> scenePoints;

        int matches_size = matches.size();

        for (int i = 0; i < matches_size; i++) {

                int image1Idx = matches[i].queryIdx;
                int image2Idx = matches[i].trainIdx;

                objPoints.push_back(keyPoints1[image1Idx].pt);
                scenePoints.push_back(keyPoints2[image2Idx].pt);
        }

	cout << "Using undistorted points for homography" << endl ;

	std::vector<cv::Point2f> undistort_img1_points;
        std::vector<cv::Point2f> undistort_img2_points;
        cv::undistortPoints(objPoints, undistort_img1_points, K1, distCoeff);
        cv::undistortPoints(scenePoints, undistort_img2_points, K1, distCoeff);


         Mat H = findHomography( objPoints, scenePoints, CV_RANSAC,1 );
         cout << "Homography" << H << endl ;

         cout << "Size of original image" << image1.size() << endl ;
         cout << "Type of original image" << image1.type() << endl ;

         Size warped_image_size=image1.size();

         Mat warped_image1=Mat(warped_image_size,image1.type());
         warpPerspective(image1, warped_image1, H, warped_image_size);
         imshow("warped_image_1",warped_image1);

         cout << "Size of warped image" << warped_image1.size() << endl ;
         cout << "Type of warped image" << warped_image1.type() << endl ;

        vector<Mat> rotation_matrices;
        vector<Mat> translation_matrices ;
        vector<Mat> normals ;

        cout << "Decomposing homography to R and T" << endl ;


        decomposeHomographyMat(H, K1, rotation_matrices,translation_matrices,normals);

        cout << "Decomposed homography to R and T" << endl ;

	Mat Rotation_1 = rotation_matrices[0];
	Mat Rotation_2 = rotation_matrices[1];
	cout << "\nR1=" << Rotation_1 << endl ;
	 cout << "\nR2=" << Rotation_2  << endl ;

	Mat Translation_1 = translation_matrices[0];
	Mat Translation_2 = translation_matrices[1];

    cout << "\nT1=" << Translation_1 <<  endl ;
    cout << "\nT2=" << Translation_2 << endl ;


	Mat Normal_1=normals[0];
	Mat Normal_2=normals[1];
	Mat Normal_3=normals[2];
	Mat Normal_4=normals[3];



        cout << "\nN1=" << Normal_1 << endl ;
        cout << "\nN2=" << Normal_2 << endl ;
        cout << "\nN3=" << Normal_3 << endl ;
        cout << "\nN4=" << Normal_4 << "\n" <<  endl ;


        float z_1=Normal_1.at<double>(2,0);
        float z_2=Normal_2.at<double>(2,0);
        float z_3=Normal_3.at<double>(2,0);
        float z_4=Normal_4.at<double>(2,0);
    
        cout << "Normal type" << Normal_1.type() << "Values" << Normal_1 << endl ;
	float depths[4];
	depths[0]=z_1;
	depths[1]=z_2;
	depths[2]=z_3;
	depths[3]=z_4;


	//maximum depth with positive sign
	int index_max=find_maximum_depth_index(depths,4);
	
	// maximum absolute depth with negative sign
	int index_min=find_minimum_depth_index(depths,4);

	int index=index_max ;

	cout << "Using index " << index << " for validR and validT " << endl ; 

        Mat validR = Mat(3, 3, CV_32F);
        Mat validT = Mat(3, 1, CV_32F);

	 if(index == 0){

                validR=Rotation_1;
                validT=Translation_1;
                cout << "\nvalid R\n" << validR ;
                cout << "\nvalid T\n" << validT ;

        }else if (index == 1){

                validR=Rotation_1;
                validT=Translation_2;
                cout << "\nvalid R\n" << validR ;
                cout << "\nvalid T\n" << validT ;

        }else if (index ==2){

                validR=Rotation_2;
                validT=Translation_1;
                cout << "\nvalid R\n" << validR ;
                cout << "\nvalid T\n" << validT ;

        } else if(index== 3)
        {
                validR=Rotation_2;
                validT=Translation_2 ;
                cout << "\nvalid R\n" << validR ;
                cout << "\nvalid T\n" << validT ;
        }
	

	float rot1_roll ;float rot1_pitch ; float rot1_yaw ;
        rod2rpy(Rotation_1.t(),&rot1_roll,&rot1_pitch,&rot1_yaw);

        cout << "\nRotation1 Roll:" << rot1_roll << " Pitch:" << rot1_pitch << " Yaw:" << rot1_yaw << endl ;

        float rot2_roll ;float rot2_pitch ; float rot2_yaw ;
        rod2rpy(Rotation_2.t(),&rot2_roll,&rot2_pitch,&rot2_yaw);

        cout << "\nRotation2 Roll:" << rot2_roll << " Pitch:" << rot2_pitch << " Yaw:" << rot2_yaw << endl ;


        transform_cordinates_and_send(validR, validT);
	return 1 ;
                                     
}


int run_eight_point_algo_on_matches1(vector<DMatch> matches,vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, Mat image1, Mat image2) {
	std::cout << "\ngoing to run 8 point algorithm to determine rotation and translation" << std::endl;

	std::vector<Point2f> objPoints;
	std::vector<Point2f> scenePoints;
	std::vector<Point2f> img1InlierPoints;
	std::vector<Point2f> img2InlierPoints;

	int matches_size = matches.size();


	for (int i = 0; i < matches_size; i++) {
		int image1Idx = matches[i].queryIdx;
		int image2Idx = matches[i].trainIdx;
		objPoints.push_back(keyPoints1[image1Idx].pt);
		scenePoints.push_back(keyPoints2[image2Idx].pt);
	}

	 Mat E, R, T, mask;

	 E=	findEssentialMat(objPoints, scenePoints, K1, RANSAC, 0.999, 1.0);

	 cout << "Essential Matrix E: " << E << endl ;

	 recoverPose(E, objPoints, scenePoints, K1, R, T, mask);

	 cout << "\nPose Recovered" << " \n Rotation:: " << R << "\nTranslation " << T << endl ;

     Mat R1_d,R2_d,T_d;
     cout << "Decomposing essential matrix to R and T" << endl ;
	 decomposeEssentialMat( E, R1_d, R2_d,T_d );

     cout << "\nR1=" << R1_d << "\nR2=" << R2_d << "\nT1=" << T_d << "\nT2=" << -T_d <<  endl ;


	 Mat validR=R;
	 Mat validT=T ;

	 cout << "\nValidR=" << validR << "\nValidT=" << T << endl ;
	 transform_cordinates_and_send(validR, validT);
	 return 1;


}


int run_eight_point_algo_on_matches(vector<DMatch> matches,
		vector<KeyPoint> keyPoints1, vector<KeyPoint> keyPoints2, Mat image1,
		Mat image2) {
	std::cout
			<< "\ngoing to run 8 point algorithm to determine rotation and translation"
			<< std::endl;

	//-- Localize the object
	std::vector<Point2f> objPoints;
	std::vector<Point2f> scenePoints;
	std::vector<Point2f> img1InlierPoints;
	std::vector<Point2f> img2InlierPoints;

//	CvMat* points1Matrix;
//	CvMat* points2Matrix;

	int matches_size = matches.size();

//	int matches_size=8 ;
//	points1Matrix = cvCreateMat(2, matches_size, CV_32F);
//	points2Matrix = cvCreateMat(2, matches_size, CV_32F);

	for (int i = 0; i < matches_size; i++) {

		int image1Idx = matches[i].queryIdx;
		int image2Idx = matches[i].trainIdx;

		objPoints.push_back(keyPoints1[image1Idx].pt);
		scenePoints.push_back(keyPoints2[image2Idx].pt);

//		cvmSet(points1Matrix, 0, i, keyPoints1[image1Idx].pt.x);
//		cvmSet(points1Matrix, 1, i, keyPoints1[image1Idx].pt.y);

//		cvmSet(points2Matrix, 0, i, keyPoints2[image1Idx].pt.x);
//		cvmSet(points2Matrix, 1, i, keyPoints2[image1Idx].pt.y);

	}

//	cout << "\nPoints1 Matrix Size Rows::" << points1Matrix->rows << "\nCols::"
//			<< points1Matrix->cols << endl;

//	CvMat* fundMatr;
//	CvMat* status ;

//	fundMatr = cvCreateMat(3, 3, CV_32F);
//	status = cvCreateMat(1,matches_size,CV_8UC1);

	//see opencv manual for other options in computing the fundamental matrix
//	int num = cvFindFundamentalMat(points1Matrix, points2Matrix, fundMatr,
//	CV_FM_RANSAC, 1.0, 0.9999,status);

//		cv::Mat fMatrix=fundMatr ;



	//printf("\nFundamental matrix was found\n ");
	//cout << fMatrix << "\n" << endl ;
////////////////////////////////////////////////
	std::vector<cv::Point2f> undistort_img1_points;
	std::vector<cv::Point2f> undistort_img2_points;
	std::vector<float> zerodist;
	cv::undistortPoints(objPoints, undistort_img1_points, K1, distCoeff);
	cv::undistortPoints(scenePoints, undistort_img2_points, K1, distCoeff);

	vector<uchar> states, dist_states, undist_states;
	Mat fMatrix1 = findFundamentalMat(objPoints, scenePoints, FM_RANSAC, 1.0,
			0.9999, dist_states);
	Mat fMatrixUndistort = findFundamentalMat(undistort_img1_points,
			undistort_img2_points, FM_RANSAC, 1.0, 0.9999, undist_states);
//	Mat hMatrix1=findHomography(objPoints, scenePoints, CV_RANSAC, 1.0);
	Mat fMatrix2;
	Mat fMatrixUndistort2;
	fMatrix1.convertTo(fMatrix2, CV_32F);
	fMatrixUndistort.convertTo(fMatrixUndistort2, CV_32F);
	//cout << "\nF computed ::" << fMatrix2.type() << "\n" << fMatrix2 << endl;

	//cout << "\nF Undistorted computed ::" << fMatrixUndistort2 << endl;
//	cout<<" \nH computed :: " << hMatrix1<<endl;
//	Mat poseFromH;
//	cameraPoseFromHomography(hMatrix1, poseFromH);
//	cout<<"Pose from H ::"<<poseFromH<<endl; 

	states = dist_states;
	for (int i = 0; i < states.size(); i++) {
		if (states[i] == 1) {
			img1InlierPoints.push_back(objPoints[i]);
			img2InlierPoints.push_back(scenePoints[i]);
		}
	}

	//cout << " Num Inlier Points : " << img2InlierPoints.size() << endl;
	//cout << "\nWill find rotation and traslation from fundamental matrix"
	//		<< endl;

	//std::cout << "\nCamera Intrinsic Matrix:: " << K1 << "\n" << std::endl;

	//std::cout << "\nType of fundamental Matrix" << fMatrix.type() << "\n Type of camera matrix" << cameraIntrinsicMatrix.type() << std::endl ;

//		drawEpilines(fMatrix2,objPoints, scenePoints,image1,image2 );		   				

	std::string title = "Epilines";

	std::cout << "Will draw epilines" << std::endl;
	drawEpipolarLines<float, float>(title, fMatrix2, image1, image2,
			img1InlierPoints, img2InlierPoints);

	Mat hard_F(3, 3, CV_32F, Scalar(0));
	hard_F.at<float>(0, 2) = -0.0072;
	hard_F.at<float>(1, 2) = -0.0250;
	hard_F.at<float>(2, 0) = 0.0041;
	hard_F.at<float>(2, 1) = 0.0172;
	hard_F.at<float>(2, 2) = 0.9995;

	Mat essentialMatrix = K1.t() * fMatrix2 * K1;
	//	essentialMatrix = fMatrixUndistort2;
	std::cout << "UnNormalised Essential matrix" << essentialMatrix
			<< std::endl;
	SVD decomp = SVD(essentialMatrix);
//
	Mat U = decomp.u;
	Mat VT = decomp.vt; //(transpose of V)
	Mat W = decomp.w;
	float det_U = determinant(U);
	float det_VT = determinant(VT);

	cout << "\nDeterminant of U:: " << det_U << ", Determinant of VT ::"
			<< det_VT << endl;
	if (det_U < 0 || det_VT < 0) {
		cout << "will do svd for negated essential matrix" << endl;
		decomp = SVD(-essentialMatrix);
		U = decomp.u;
		VT = decomp.vt;
		W = decomp.w;

		det_U = determinant(U);
		det_VT = determinant(VT);
		cout << "\nDeterminant of U:: " << det_U << ", Determinant of VT ::"
				<< det_VT << endl;

	}

	std::cout << "\nMatrix U after decomposition:: " << U << std::endl;
	std::cout << "\nMatrix VT after decomposition:: " << VT << std::endl;
	std::cout << "\nDiagonal matrix W::" << W << "\n" << std::endl;

//		//Diogonal matrix
	Mat D(3, 3, CV_32F, Scalar(0));
	//D.at<float>(0, 0) = W.at<float>(0, 0);
	float e = (W.at<float>(0.0) + W.at<float>(1.1)) / 2.0;
	D.at<float>(0, 0) = e;
	//D.at<float>(1, 1) = W.at<float>(0, 1);
	D.at<float>(1, 1) = e;
	//D.at<float>(2, 2) = W.at<float>(0, 2);
	D.at<float>(2, 2) = 0;

	std::cout << "\nMatrix D::" << D << std::endl;

	Mat finalE;
	Mat hard_finalE(3, 3, CV_32F, Scalar(0));
	hard_finalE.at<float>(0, 0) = 0.3939;
	hard_finalE.at<float>(0, 1) = 13.7709;
	hard_finalE.at<float>(0, 2) = -0.2684;
	hard_finalE.at<float>(1, 0) = -13.1206;
	hard_finalE.at<float>(1, 1) = 0.2425;
	hard_finalE.at<float>(1, 2) = -5.4987;
	hard_finalE.at<float>(2, 0) = 0.0068;
	hard_finalE.at<float>(2, 1) = 3.5477;
	hard_finalE.at<float>(2, 2) = -0.1088;

	finalE = U * D * VT; //not used anywhere

	SVD decomp_finalE = SVD(finalE);
	std::cout << "\nNormalised essential Matrix" << finalE << std::endl;
	// Rz and its transpose is used

	U = decomp_finalE.u;
	VT = decomp_finalE.vt; //(transpose of V)

	Mat Rz(3, 3, CV_32F, Scalar(0));
	Rz.at<float>(0, 1) = -1;
	Rz.at<float>(1, 0) = 1;
	Rz.at<float>(2, 2) = 1;

	cout << "\nRz " << Rz << "\nRz transpose " << Rz.t() << std::endl;

	Mat Rotation_1 = U * Rz * VT;
	cout << "\nR1.t()*R1 = " << Rotation_1.t() * Rotation_1;
	if (determinant(Rotation_1) < 0) {
		cout << "\nMinus of R1.t()*R1 = " << Rotation_1.t() * Rotation_1;
		Rotation_1 = -Rotation_1;
	}
	double yawAngle1 = getYawAngleFromRotationMatrix(Rotation_1);

	Mat Z(3, 3, CV_32F, Scalar(0));
	Z.at<float>(0, 1) = 1;
	Z.at<float>(1, 0) = -1;
	cout << "\n Computed Rotation1:: " << Rotation_1 << "\nYaw Angle:: "
			<< yawAngle1 << endl;
	Mat T_hat1 = U * Z * U.t();
	cout << "\nComputed Transation matrix1:: " << T_hat1 << std::endl;

	Mat T1 = Mat(3, 1, CV_32F);
	T1.at<float>(0, 0) = -T_hat1.at<float>(1, 2);
	T1.at<float>(0, 1) = T_hat1.at<float>(0, 2);
	T1.at<float>(0, 2) = -T_hat1.at<float>(0, 1);

	cout << "\nTranslation vector1:: " << T1 << endl;

	Mat Rotation_2 = U * Rz.t() * VT;
	cout << "\nR2.t()*R2 = " << Rotation_2.t() * Rotation_2;
	if (determinant(Rotation_2) < 0) {
		cout << "\nR2.t()*R2 = " << Rotation_2.t() * Rotation_2;
		Rotation_2 = -Rotation_2;
	}
	double yawAngle2 = getYawAngleFromRotationMatrix(Rotation_2);

	cout << "Computed rotation2:: " << Rotation_2 << "\nYaw Angle2:: "
			<< yawAngle2 << endl;
	;
	Mat T_hat2 = U * Z.t() * U.t();
	cout << "\nComputed Transation matrix2:: " << T_hat2 << std::endl;

	Mat T2 = Mat(3, 1, CV_32F);

	T2.at<float>(0, 0) = -T_hat2.at<float>(1, 2);
	T2.at<float>(0, 1) = T_hat2.at<float>(0, 2);
	T2.at<float>(0, 2) = -T_hat2.at<float>(0, 1);

	cout << "\nTranslation vector2:: " << T2 << endl;

	Mat validR = Mat(3, 3, CV_32F);
	Mat validT = Mat(3, 1, CV_32F);



	//	std::cout<<"R1 = "<<Rotation_1<<endl;
	//	std::cout<<"R2 = "<<Rotation_2<<endl;
	//	std::cout<<"T1 = "<<T1<<endl;
	//	std::cout<<"T2 = "<<T2<<endl;
	validateRandT2(K1, K1, distCoeff, Rotation_1, Rotation_2, T1, T2, image1,
			image2, img1InlierPoints, img2InlierPoints, validR, validT);

	//Mat new_camera_center = -validR.t() * validT;
	//float x = new_camera_center.at<float>(0, 0);
	//float y = new_camera_center.at<float>(1, 0);
	//float z = new_camera_center.at<float>(2, 0);

	//	cout << "size of M" << new_camera_center.size() << " Sending x" << x << " y"
	//		<< y << " z" << z << endl;

	//gotoCommand(x, y, z, 0);

	//double yaw = getYawAngleFromRotationMatrix(validR);

	std::cout << "R1 = " << Rotation_1 << endl;
	std::cout << "R2 = " << Rotation_2 << endl;
	std::cout << "T1 = " << T1 << endl;
	std::cout << "T2 = " << T2 << endl;

	//std::cout << "Yaw:" << yaw << std::endl;
	std::cout << "T =" << validT << std::endl;
	std::cout << "R =" << validR << std::endl;

	float rot1_roll ;
	float rot1_pitch ;
	float rot1_yaw ;
	rod2rpy(Rotation_1.t(),&rot1_roll,&rot1_pitch,&rot1_yaw);

	//cout << "Rotation1 r:" << rot1_roll << " p:" << rot1_pitch << " y:" << rot1_yaw << endl ;

	float rot2_roll ;
	float rot2_pitch ;
	float rot2_yaw ;
	rod2rpy(Rotation_2.t(),&rot2_roll,&rot2_pitch,&rot2_yaw);

	cout << "Rotation2 r:" << rot2_roll << " p:" << rot2_pitch << " y:" << rot2_yaw << endl ;


	//validR = Mat::eye(3,3,CV_32F);

	//validT = (cv::Mat_<double>(3,1) << 1,0,0);

	//cout << "Testing for validR::" << validR << "validT" << validT << endl ;

	transform_cordinates_and_send(validR, validT);
	return 1;

//	} else {
//		printf("Fundamental matrix was not found\n");

//	}

	//			  cout << "Will find homography NOW" << endl ;
	//			  Mat H = findHomography( obj, scene, CV_RANSAC );
	//
	//			  std::vector<Point2f> obj_corners(4);
	//			  obj_corners[0] = cvPoint(0,0);
	//			  obj_corners[1] = cvPoint( gray1.cols, 0 );
	//			  obj_corners[2] = cvPoint( gray1.cols, gray1.rows );
	//			  obj_corners[3] = cvPoint( 0, gray1.rows );
	//			  std::vector<Point2f> scene_corners(4);
	//
	//			  perspectiveTransform( obj_corners, scene_corners, H);
	//			  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
	//			  line( img_matches, scene_corners[0] + Point2f( gray1.cols, 0), scene_corners[1] + Point2f( gray1.cols, 0), Scalar(0, 255, 0), 4 );
	//			  line( img_matches, scene_corners[1] + Point2f( gray1.cols, 0), scene_corners[2] + Point2f( gray1.cols, 0), Scalar( 0, 255, 0), 4 );
	//			  line( img_matches, scene_corners[2] + Point2f( gray1.cols, 0), scene_corners[3] + Point2f( gray1.cols, 0), Scalar( 0, 255, 0), 4 );
	//			  line( img_matches, scene_corners[3] + Point2f( gray1.cols, 0), scene_corners[0] + Point2f( gray1.cols, 0), Scalar( 0, 255, 0), 4 );
	//
	////			    //-- Show detected matches
	////
	//		imshow( "matches_and_object_detection", img_matches );

}

void update_matched_image_in_status_figure(Mat matched_image){

	IplImage matched_image_ptr = (IplImage)matched_image;
	//IplImage img2 = (IplImage)image2;


	cvSetImageROI(status_image, cvRect(0, matched_image_ptr.height+50,matched_image_ptr.width, matched_image_ptr.height) );
	cvCopy(&matched_image_ptr,status_image,NULL);
	cvResetImageROI(status_image);

	cv::Mat dstMat = cv::cvarrToMat(status_image);
	imshow("status_image",dstMat);


}

void reset_background_with_color(int start_x, int start_y, int width,int height,int color){
	IplImage* img1_zeros=cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
	if(color == 0){
		cvSet(img1_zeros, cvScalar(0,0,0));
	}else if (color == 1){
		cvSet(img1_zeros, cvScalar(255,0,0));

	}else if (color == 2){
		cvSet(img1_zeros, cvScalar(0,255,0));

	}

	cvSetImageROI(status_image, cvRect(start_x, start_y,width,height) );
	cvCopy(img1_zeros,status_image,NULL);
	cvResetImageROI(status_image);
}

void set_target_achieved_status(){
	reset_background_with_color((status_image_width/2)-1,0 ,status_image_width/2,50,2);

//	string status_msg="TARGET ACHIEVED";
	cv::Mat dstMat = cv::cvarrToMat(status_image);
//
//	putText(dstMat,status_msg,cvPoint(80,30), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,0),1.5);
	imshow("status_image",dstMat);

}

void update_command_sent_status(string command_sent){
	reset_background_with_color((status_image_width/2)-1,0 ,status_image_width/2,50,0);

	string status_msg="Command Sent: "+ command_sent;
	cv::Mat dstMat = cv::cvarrToMat(status_image);

	putText(dstMat,status_msg,cvPoint(650,10), FONT_HERSHEY_PLAIN, 0.8, Scalar(0,0,255),1.5);
	imshow("status_image",dstMat);

}

void update_command_executed_status(string command_executed){
	reset_background_with_color((status_image_width/2)-1,25,status_image_width/2,25,0);
	string status_msg="Command Executed: "+ command_executed;

	cv::Mat dstMat = cv::cvarrToMat(status_image);
	putText(dstMat,status_msg,cvPoint(650,40), FONT_HERSHEY_PLAIN, 0.8, Scalar(0,255,0),1.5);
	imshow("status_image",dstMat);

}

void update_quad_status_message(string status){

	reset_background_with_color(0,0,status_image_width/2,50,0);
	cv::Mat dstMat = cv::cvarrToMat(status_image);

	putText(dstMat,status,cvPoint(30,30), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,0,0),2);
	imshow("status_image",dstMat);

}


void update_current_image_in_status_figure(Mat image1){


	//cout << "updating image status" << endl ;
	IplImage img1 = (IplImage)image1;

	// Copy first image to dst
	cvSetImageROI(status_image, cvRect(0, 50,img1.width,img1.height) );
	cvCopy(&img1,status_image,NULL);
	cvResetImageROI(status_image);

	cv::Mat dstMat = cv::cvarrToMat(status_image);
	putText(dstMat,"Quadcopter View",cvPoint(30,350), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255),2);

	imshow("status_image",dstMat);



}

void update_target_image_in_status_figure(Mat image2){


	IplImage img2 = (IplImage)image2;

	// Copy second image to dst
	cvSetImageROI(status_image, cvRect(img2.width, 50,img2.width,img2.height) );
	cvCopy(&img2,status_image,NULL);
	cvResetImageROI(status_image);


	cv::Mat dstMat = cv::cvarrToMat(status_image);
	putText(dstMat,"Target View",cvPoint(670,350), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255),2);

	imshow("status_image",dstMat);

}


void enable_debug_mode(const std_msgs::String::ConstPtr& msg ){

	debug_mode=true;

	char* target_image_name= (char*)msg->data.c_str();
	cout << "image1 is " << target_image_name << endl ;

	char target_image_full_path[200] = "/usr/prakt/w041/" ;
	strcat(target_image_full_path,target_image_name);
	cout << "Target image full path " << target_image_full_path << endl ;

	Mat	image1 = imread(target_image_full_path,CV_LOAD_IMAGE_COLOR);

	if (!image1.data)                             // Check for invalid input
	{
			 cout << "Could not open or find the image1" << std::endl;

	}

	Mat undistort_image1;
	undistort(image1,undistort_image1,K1,distCoeff);
	image1 = undistort_image1 ;

	update_current_image_in_status_figure(image1);
	calculate(image1);
}

void targetImageCallback(const std_msgs::String::ConstPtr& msg ){

	pthread_mutex_lock(&image2lock);

	image_2_reached=false ;
	keyboardEventReceived=true ;

	reset_background_with_color((status_image_width/2)-1,0 ,status_image_width/2,50,0);
	char* target_image_name= (char*)msg->data.c_str();
	cout << "Target image is " << target_image_name << endl ;
//
	char target_image_full_path[200] = "/usr/prakt/w041/" ;
//
	strcat(target_image_full_path,target_image_name);
	cout << "Target image full path " << target_image_full_path << endl ;

	image2 = imread(target_image_full_path,CV_LOAD_IMAGE_COLOR);


	Mat undistort_image2;
	undistort(image2, undistort_image2, K1, distCoeff);
	image2 = undistort_image2;

	update_target_image_in_status_figure(image2);

	pthread_mutex_unlock(&image2lock);




}

bool check_if_within_theshold(float cur_pos,float target_pos,float threshold){
	if(fabs(cur_pos - target_pos) < threshold){
		return true;
	}

return false;
}

string getDroneStatus(int status){
	string status_string ;
//# 0: Unknown, 1: Init, 2: Landed, 3: Flying, 4: Hovering, 5: Test
//# 6: Taking off, 7: Goto Fix Point, 8: Landing, 9: Looping
//# Note: 3,7 seems to discriminate type of flying (isFly = 3 | 7)

	//cout << "finding status string for status" << status  << endl ;
	if (status == 0){
		status_string = "UNKNOWN";
	}else if(status == 1){
		status_string = "Init";
	}else if(status == 2){
		status_string = "Landed";
	}else if(status == 3){
		status_string = "Flying";
	}else if(status == 4){
		status_string = "Hovering";
	}else if(status == 5){
		status_string = "Test";
	}else if(status == 6){
		status_string = "Taking off";
	}else if(status == 7){
		status_string = "Goto Fix Point";
	}else if(status == 8){
		status_string = "Landing";
	}else if(status == 9){
		status_string = "Looping";
	}
	//cout << "finding status string for status" << status_string  << endl ;

	return status_string;
}

void droneposeCb(const tum_ardrone::filter_stateConstPtr statePtr) {
	//cout << "callback received for drone pose!!" << endl;


	quad_poseConstPtr = statePtr;

	float x=quad_poseConstPtr->x;
	float y=quad_poseConstPtr->y;
	float z=quad_poseConstPtr->z;
	float yaw=quad_poseConstPtr->yaw;

	float pos_threshold=0.3;

	//cout <<"X: " << x << " positionT.x: " << positionT.x << " Diff: " << fabs(x - positionT.x) << endl ;
	//cout <<"Y: " << y << " positionT.y: " << positionT.y << " Diff: " << fabs(y - positionT.y) << endl ;
	//cout <<"Z: " << z << " positionT.z: " << positionT.z << " Diff: " << fabs(z - positionT.z) << endl ;

	if(check_if_within_theshold(x,positionT.x,pos_threshold) && check_if_within_theshold(y,positionT.y,pos_threshold)
			&& check_if_within_theshold(z,positionT.z,pos_threshold) ){

		//cout << "Received pose is within threshold" << endl ;
		if(!target_pose_achieved){
			//cout << "Setting target pose achieved to true" << endl ;
			target_pose_achieved=true ;
			target_pose_achieved_time = ros::Time::now().toSec();
		}else {
			double current_time=ros::Time::now().toSec();
			double diff_time=current_time-target_pose_achieved_time;
			//cout << "Diff time for pose within target pose" << diff_time << endl ;
			if(diff_time >= 5 ){
				if(!image_2_reached){
					char buf[50];
					sprintf(buf, "%f %f %f %f", statePtr->x, statePtr->y, statePtr->z, statePtr->yaw);
					string status_msg(buf) ;
				    update_command_executed_status(status_msg);
				}

				ros::Duration d(1);
				//d.sleep();
				keyboardEventReceived=true ;
			}

		}



//
//
	}else {
		//cout << "Received pose is out of threshold" << endl;
		target_pose_achieved=false ;
	}

	int drone_state=quad_poseConstPtr->droneState;

	string status_msg=getDroneStatus(drone_state);
	update_quad_status_message(status_msg);

}

void reset_momentum(){
	current_momentum.x = 1.0;
	current_momentum.y = 1.0;
	current_momentum.z = 1.0;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "image_listener");


	ros::NodeHandle nh;


	cv::namedWindow("warped_image_1", cv::WINDOW_NORMAL);
	cv::startWindowThread();


	cv::namedWindow("status_image", cv:: WINDOW_AUTOSIZE );
	cv::startWindowThread();

	reset_momentum();

	if (CV_MAJOR_VERSION < 3) {
		cout << "less than 3" << endl;
	} else {
		cout << "MORE than 3" << endl;
	}


	status_image=cvCreateImage(cvSize(status_image_width,status_image_height),IPL_DEPTH_8U,3);

	cv::Mat dstMat = cv::cvarrToMat(status_image);
	putText(dstMat,"Quadcopter View",cvPoint(30,350), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255),2);
	putText(dstMat,"Target View",cvPoint(670,350), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255),2);

	imshow("status_image",dstMat);


	vector<float> distCoeff;// = [-0.525878 0.321315 0.000212 -0.000429 0.000000];
		// 1
/*		distCoeff.push_back(-0.525878);
		distCoeff.push_back(0.321315);
		distCoeff.push_back(0.000212);
		distCoeff.push_back(-0.000429);
		distCoeff.push_back(0.000000);
*/
		//2
//		distCoeff.push_back(-0.521768);
//		distCoeff.push_back(0.276146);
//		distCoeff.push_back(-0.001513);
//		distCoeff.push_back(0.000532);
//		distCoeff.push_back(0.000000);
//
//
//	 	focal_length_x = 575.426009;
//	 	focal_length_y = 576.174707;
//	 	c_x = 300.303245;
//	 	c_y = 187.30014;
//		scew = 0.0;
		//3
		//[571.230731, 0, 332.648207, 0, 571.26822, 171.726625, 0, 0, 1]
		//[-0.517675, 0.264322, -0.001992, 0.0005679999999999999, 0]
		distCoeff.push_back(-0.517675);
		distCoeff.push_back(0.264322);
		distCoeff.push_back(-0.001992);
		distCoeff.push_back(0.0005679);
		distCoeff.push_back(0.000000);


	 	focal_length_x = 571.230731;
	 	focal_length_y = 571.26822;
	 	c_x = 332.648207;
	 	c_y = 171.726625;
		scew = 0.0;

		K1.at<float>(0, 0) = focal_length_x;
		K1.at<float>(0, 1) = scew;
		K1.at<float>(0, 2) = c_x;

		K1.at<float>(1, 1) = focal_length_y;
		K1.at<float>(1, 2) = c_y;
	
		K1.at<float>(2, 2) = 1;

		image_transport::ImageTransport it(nh);

		image_transport::Subscriber sub = it.subscribe("/ardrone/front/image_raw",	1, imageCallback);


		pub_com = nh.advertise<std_msgs::String>("tum_ardrone/com", 50);
		pub_clear_com = nh.advertise<std_msgs::String>("drone_autopilot/clearCommands", 50);
		ros::Subscriber dronepose_sub = nh.subscribe("ardrone/predictedPose", 10, droneposeCb);
		ros::Subscriber sub1 = nh.subscribe("photo_goal/force", 1, keyboardCallback);
		ros::Subscriber target_image_sub = nh.subscribe("photo_goal/target_image", 10, targetImageCallback);

		ros::Subscriber enable_debug_mode_sub = nh.subscribe("photo_goal/source_image", 10, enable_debug_mode);
		ros::spin();




	//	Mat image1=imread("/usr/prakt/w041/video_seq_images3_z/frame0088.jpg", CV_LOAD_IMAGE_COLOR) ;
	//
	//	if (!image1.data)                             // Check for invalid input
	//	{
	//			 cout << "Could not open or find the image1" << std::endl;
	//                			return 0;
	//	}


//	Mat undistort_image1;
//	undistort(image1,undistort_image1,K1,distCoeff);
//	image1 = undistort_image1 ;



//
//	//Size size(640, 360);                        //the dst image size,e.g.100x100
//	// resize(input, image1, size);                             //resize image
//
//	cv::imshow("handy_image", image2);
//
//	cout << "going to subscibe" << endl;





	//calculate(image1);
	//Mat validR = Mat::eye(3, 3, CV_32F);
	//validR = rpy2rod(30,30, 0);

	//Mat validT = (cv::Mat_<float>(3, 1) << -1, 0, 0);

  //cout << "Testing for validR::" << validR << "validT" << validT << endl ;

	//setConsideredPose();

//	ros::Duration d(1);
//	cout << "Bef" << endl ;
//	ros::spinOnce();
//	d.sleep();
//	ros::spinOnce();
//
//	cout << "Aft" << endl ;
	//transform_cordinates_and_send(validR, validT);



}

//Mat output;
//drawKeypoints(gray, keypoints, output);
// namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.

// imshow( "Display window", output );
// imshow( "Display window", output );                   // Show our image inside it.


//	cout << "\nPoints1 Matrix Size Rows::" << points1Matrix->rows << "\nCols::"
//			<< points1Matrix->cols << endl;

//	CvMat* fundMatr;
//	CvMat* status ;

//	fundMatr = cvCreateMat(3, 3, CV_32F);
//	status = cvCreateMat(1,matches_size,CV_8UC1);

	//see opencv manual for other options in computing the fundamental matrix
//	int num = cvFindFundamentalMat(points1Matrix, points2Matrix, fundMatr,
//	CV_FM_RANSAC, 1.0, 0.9999,status);

//		cv::Mat fMatrix=fundMatr ;



	//printf("\nFundamental matrix was found\n ");
	//cout << fMatrix << "\n" << endl ;
////////////////////////////////////////////////

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/distortion_models.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

ros::Publisher pub;
tf::TransformListener *listener;

void imageCallback(const sensor_msgs::ImageConstPtr& rgb_img,
		const sensor_msgs::ImageConstPtr& depth_img) {
	try {

		cv::Mat rgb_cv_img = cv_bridge::toCvShare(rgb_img, "bgr8")->image;
		cv::Mat depth_cv_img = cv_bridge::toCvShare(depth_img)->image;

//	ROS_ERROR("%s", rgb_cv_img->header.frame_id.c_strt());

		float fx = 525.0; // focal length x
		float fy = 525.0; //focal length y
		float cx = 319.5; // optical center x
		float cy = 239.5;  // optical center y
		float factor = 1.0;

		pcl::PointCloud<pcl::PointXYZRGB> cloud;

		cloud.width = depth_cv_img.rows;
		cloud.height = depth_cv_img.cols;
		cloud.points.resize(cloud.width * cloud.height);
		int count = 0;

		cv::Mat channels[3];
		cv::split(rgb_cv_img, channels);

		ros::Time msg_time = rgb_img->header.stamp;
		//      std::string f_str = to_str(msg_time);
		//      ROS_ERROR("%s",f_str);
		tf::StampedTransform transform;
		try {
			listener->waitForTransform("/world", "/openni_rgb_optical_frame",
					msg_time, ros::Duration(4.0));
			listener->lookupTransform("/world", "/openni_rgb_optical_frame",
					msg_time, transform);

		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			return;
		}

		for (int y = 0; y < depth_cv_img.rows; y = y + 1) {
			for (int x = 0; x < depth_cv_img.cols; x = x + 1) {
				float depth_value = depth_cv_img.at<float>(y, x);
				float Z = depth_value / factor;
				float X = (x - cx) * Z / fx;
				float Y = (y - cy) * Z / fy;
				//These cordinates are in kinect frame..changing the to world frame using transforma
				cloud.points[count].x = X;
				cloud.points[count].y = Y;
				cloud.points[count].z = Z;
				cloud.points[count].r = channels[2].at<uchar>(y, x);
				cloud.points[count].g = channels[1].at<uchar>(y, x);
				cloud.points[count].b = channels[0].at<uchar>(y, x);
				count = count + 1;
			}
		}

		pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
		pcl_ros::transformPointCloud(cloud, transformed_cloud, transform);

		sensor_msgs::PointCloud2 cloudMsg;
		pcl::toROSMsg(transformed_cloud, cloudMsg);
		cloudMsg.header.frame_id = "test_world";

		pub.publish(cloudMsg);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert to 'rgb8'.");
	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;

	message_filters::Subscriber<sensor_msgs::Image> sub1(nh,
			"/camera/rgb/image_color", 1000);
	message_filters::Subscriber<sensor_msgs::Image> sub2(nh,
			"/camera/depth/image", 1000);

	listener = new tf::TransformListener(ros::Duration(5), true);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
			sensor_msgs::Image> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1,
			sub2);

	sync.registerCallback(boost::bind(&imageCallback, _1, _2));

	image_transport::ImageTransport it(nh);
	pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud_topic", 1);

	ros::spin();
}

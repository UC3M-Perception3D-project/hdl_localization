#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>

ros::Publisher pub_cloud;
ros::Publisher pub_imu;

std_msgs::Header last_odom_header;

typedef message_filters::sync_policies::ApproximateTime <nav_msgs::Odometry, nav_msgs::Odometry> inertial_odom_kf_odom;
// Synchronizer
message_filters::Synchronizer<inertial_odom_kf_odom> *odom_sync; // Callback

std::ofstream file_kf_vs_inertial;
void callback_sync ( const nav_msgs::OdometryConstPtr &odom_inertial_msg, const nav_msgs::OdometryConstPtr &odom_kf_msg )
{
	file_kf_vs_inertial << odom_inertial_msg->pose.pose.position.x << " " << odom_inertial_msg->pose.pose.position.y << " " << odom_inertial_msg->pose.pose.position.z << " "
		 << odom_inertial_msg->pose.pose.orientation.w << " " << odom_inertial_msg->pose.pose.orientation.x << " " << odom_inertial_msg->pose.pose.orientation.y << " " << odom_inertial_msg->pose.pose.orientation.z << "\n";
	file_kf_vs_inertial << odom_kf_msg->pose.pose.position.x << " " << odom_kf_msg->pose.pose.position.y << " " << odom_kf_msg->pose.pose.position.z << " "
		 << odom_kf_msg->pose.pose.orientation.w << " " << odom_kf_msg->pose.pose.orientation.x << " " << odom_kf_msg->pose.pose.orientation.y << " " << odom_kf_msg->pose.pose.orientation.z << "\n";
	// file_kf_vs_inertial.close();
}



void pcdCallback (const sensor_msgs::PointCloud2& msg)
{
  sensor_msgs::PointCloud2 new_pc_msg;

  pcl::PointCloud <pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud <pcl::PointXYZI> ());
  pcl::fromROSMsg (msg, *output_cloud);
  pcl::toROSMsg ( *output_cloud, new_pc_msg );

  new_pc_msg.header.stamp.sec = last_odom_header.stamp.sec;
  new_pc_msg.header.stamp.nsec = last_odom_header.stamp.nsec;
  pub_cloud.publish ( new_pc_msg );
}

void imuCallback (const sensor_msgs::Imu& msg)
{
	sensor_msgs::Imu new_imu_msg = msg;

	new_imu_msg.header.stamp.sec = last_odom_header.stamp.sec;
  	new_imu_msg.header.stamp.nsec = last_odom_header.stamp.nsec;
	pub_imu.publish ( new_imu_msg );
}

void odomCallback ( const nav_msgs::OdometryConstPtr& odom_msg )
{
  last_odom_header = odom_msg->header;
}

std::ofstream kf_odom;
void odomKfCallback ( const nav_msgs::OdometryConstPtr& odom_msg )
{
	kf_odom << odom_msg->pose.pose.position.x << " " << odom_msg->pose.pose.position.y << " " << odom_msg->pose.pose.position.z << " "
		 << odom_msg->pose.pose.orientation.w << " " << odom_msg->pose.pose.orientation.x << " " << odom_msg->pose.pose.orientation.y << " " << odom_msg->pose.pose.orientation.z << "\n";

	kf_odom.close();
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "sync_odom_lidar");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	// -------------------------------------- //
	std::string output_path = "/home/lsi/UC3M/Percepcion_3D/Trabajo/comparaciones/kf_vs_inertial.txt";
	file_kf_vs_inertial.open(output_path);

	output_path = "/home/lsi/UC3M/Percepcion_3D/Trabajo/kalman_odom/kf_odom.txt";
	kf_odom.open(output_path);
	// -------------------------------------- //
	
	pub_cloud = nh.advertise< sensor_msgs::PointCloud2 > ("points_correct_stamp", 5);
	pub_imu = nh.advertise< sensor_msgs::Imu > ("imu_correct_stamp", 5);

	ros::Subscriber cloud_sub = nh.subscribe ("/os_cloud_node/points", 5, pcdCallback);
	ros::Subscriber imu_sub = nh.subscribe ("/os_cloud_node/imu", 5, imuCallback);
  	ros::Subscriber odom_sub = nh.subscribe ("/robot/robotnik_base_control/odom", 5, odomCallback);
	ros::Subscriber kf_sub = nh.subscribe ("/odom", 5, odomKfCallback);

	// Subscribers / Input synchronization
	message_filters::Subscriber<nav_msgs::Odometry> * odom_kf_sub = new message_filters::Subscriber <nav_msgs::Odometry> (nh, "/odom", 10);
	message_filters::Subscriber<nav_msgs::Odometry> * odom_inertial_sub = new message_filters::Subscriber <nav_msgs::Odometry> (nh, "/robot/robotnik_base_control/odom", 10);
	// Synchronizer
	odom_sync = new message_filters::Synchronizer<inertial_odom_kf_odom>( inertial_odom_kf_odom(10), *odom_inertial_sub, *odom_kf_sub );
	// Register callbacks
	odom_sync->registerCallback( boost::bind ( &callback_sync, _1, _2 ) );
	ros::spin();
	file_kf_vs_inertial.close();
	kf_odom.close();
	return 0;
}
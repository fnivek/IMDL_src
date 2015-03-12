#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <stdio.h>

ros::Publisher evil_global_pub;

void segmentCloud(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
	// Convert from ros msg
	pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*pc, *raw_pc);

	// Make vars for coeffs and indices
	pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;

	// Set up segmentator
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);		// Set random concensus model to plane
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.1);				// Set threshold to 10th of a meter

	// Segment
	seg.setInputCloud(raw_pc);
	seg.segment(*inliers, *coeffs);

	// Tell ROS whats up
	ROS_INFO("x: %f,\ty: %f,\tz: %f,\tw: %f", coeffs->values[0], coeffs->values[1], coeffs->values[2], coeffs->values[3]);

	// Output msg
	sensor_msgs::PointCloud2::Ptr out_msg(new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*raw_pc, *inliers, *ground);
	pcl::toROSMsg(*ground, *out_msg);
	evil_global_pub.publish(out_msg);

}

/************************
 * 			Main		*
 ************************/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "plane_extractor");
	ros::NodeHandle nh;
	ros::Rate update_rate(100);

	ros::Subscriber pc_listener =
			nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",
			100,
			&segmentCloud);

	evil_global_pub = nh.advertise<sensor_msgs::PointCloud2>("ground", 10);

	while(ros::ok())
	{
		update_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

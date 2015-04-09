#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <stdio.h>

ros::Publisher evil_global_pub;
ros::Publisher evil_global_pub2;

void segmentCloud(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
	// Convert from ros msg
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*pc, *raw_pc);

	// Make kd tree
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
			boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

	// Shrink in the y axis
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (raw_pc);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (0.1, 5);
	pass.filter (*indices);


	sensor_msgs::PointCloud2::Ptr out_msg(new sensor_msgs::PointCloud2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*raw_pc, *indices, *out_pc);
	pcl::toROSMsg(*out_pc, *out_msg);
	evil_global_pub2.publish(out_msg);

	// Color region grow
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud (raw_pc);
	reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (10);
	reg.setPointColorThreshold (6);
	reg.setRegionColorThreshold (5);
	reg.setMinClusterSize (200);

	// Get the cloud
	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);


	// Output msgs

	for(std::vector<pcl::PointIndices>::iterator it = clusters.begin(); it < clusters.end(); ++it)
	{
		pcl::copyPointCloud(*raw_pc, *it, *out_pc);
		pcl::toROSMsg(*out_pc, *out_msg);
		evil_global_pub.publish(out_msg);
	}

}

/************************
 * 			Main		*
 ************************/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "colored_obj_extractor");
	ros::NodeHandle nh;
	ros::Rate update_rate(100);

	ros::Subscriber pc_listener =
			nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",
			100,
			&segmentCloud);

	evil_global_pub = nh.advertise<sensor_msgs::PointCloud2>("colored_objs", 10);
	evil_global_pub2 = nh.advertise<sensor_msgs::PointCloud2>("colored_objs2", 10);

	while(ros::ok())
	{
		update_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}
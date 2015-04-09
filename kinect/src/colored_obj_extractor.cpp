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

void segmentCloud(const sensor_msgs::PointCloud2::ConstPtr& pc)
{
	// Convert from ros msg
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*pc, *raw_pc);

	// Make kd tree
	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
			boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (raw_pc);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (0.1, 5);
	//pass.setFilterLimitsNegative (true);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr greenish_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	pass.filter (*greenish_pc);

	// Output msg
	sensor_msgs::PointCloud2::Ptr out_msg(new sensor_msgs::PointCloud2);
	pcl::toROSMsg(*greenish_pc, *out_msg);
	evil_global_pub.publish(out_msg);

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
			nh.subscribe<sensor_msgs::PointCloud2>("raw_pc",
			100,
			&segmentCloud);

	evil_global_pub = nh.advertise<sensor_msgs::PointCloud2>("colored_objs", 10);

	while(ros::ok())
	{
		update_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

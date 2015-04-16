#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/PointIndices.h>
//#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>

#include <stdio.h>


class object_extractor
{
private:	// Type defs
	typedef pcl::PointXYZ point;
	typedef pcl::PointCloud<point> pointCloud;
	typedef sensor_msgs::PointCloud2 point_msg;

private:	// Vars
	double min_height_;
	double max_height_;

public:		// Vars
	ros::Publisher test_pub1_;
	ros::Publisher test_pub2_;

	ros::Subscriber raw_pc_sub_;

public:		// Funcitons
	object_extractor();

	void CloudCb(const point_msg::ConstPtr& pc);


};

object_extractor::object_extractor() :
		min_height_(0),
		max_height_(5)
{

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	raw_pc_sub_ = nh.subscribe<point_msg>("/camera/depth/points", 10, &object_extractor::CloudCb, this);

	test_pub1_ = nh.advertise<point_msg>(nh.resolveName("test_pc1"), 10);
	test_pub2_ = nh.advertise<point_msg>(nh.resolveName("test_pc2"), 10);

	std::string name = private_nh.resolveName("min_pc_height");
	private_nh.param<double>(name.c_str(), min_height_, -0.5);
	ROS_INFO("Param %s value %f", name.c_str(), min_height_);

	name = private_nh.resolveName("max_pc_height");
	private_nh.param<double>(name.c_str(), max_height_, 0.05);
	ROS_INFO("Param %s value %f", name.c_str(), max_height_);
}

void object_extractor::CloudCb(const point_msg::ConstPtr& pc)
{
	// Convert from ros msg
	pointCloud::Ptr raw_pc(new pointCloud);
	pcl::fromROSMsg(*pc, *raw_pc);

	/* Debug
	float miny = 9999999999;
	float maxy = -9999999999;
	for (pcl::PointCloud<point>::const_iterator it = raw_pc->begin(); it < raw_pc->end(); ++it)
	{
		float y = it->y;
		if(y < miny)
		{
			miny = y;
		}
		else if(y > maxy)
		{
			maxy = y;
		}
	}
	ROS_INFO("yE[%f, %f]", miny, maxy);
	*/

	// Get rid of points to low and points that are to high
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<point> pass;
	pass.setInputCloud (raw_pc);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (min_height_, max_height_);
	pointCloud::Ptr height_filtered_pc(new pointCloud);
	pass.filter (*height_filtered_pc);


	// Debug output
	point_msg::Ptr out_msg(new point_msg);
	pcl::toROSMsg(*height_filtered_pc, *out_msg);
	test_pub1_.publish(out_msg);

	// Estimate point normals
	// Make kd tree
	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
	pcl::NormalEstimation<point, pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(height_filtered_pc);
	ne.setKSearch(50);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*normals);

	// Set up segmenter
	pcl::SACSegmentationFromNormals<point, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0, 0.1);
	seg.setInputCloud(height_filtered_pc);
	seg.setInputNormals(normals);

	// Segment data
	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.segment(*inliers, *coeff);


	/* Color region grow
	// Make kd tree
	pcl::search::Search <point>::Ptr tree =
			boost::shared_ptr<pcl::search::Search<point> > (new pcl::search::KdTree<point>);
	pcl::RegionGrowingRGB<point> reg;
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
	*/
}

/************************
 * 			Main		*
 ************************/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "object_extractor");

	object_extractor obj_extract;

	ros::Rate update_rate(100);
	while(ros::ok())
	{
		update_rate.sleep();
		ros::spinOnce();
	}

	return 0;
}

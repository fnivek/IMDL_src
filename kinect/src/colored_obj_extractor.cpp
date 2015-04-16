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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

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
	double cluster_tolerance_;			//Max distance in meters between points
	int min_cluster_size_;		//Minimum number of points in a cluster
	int max_cluster_size_;		//Maximum number of points in a cluster
	double voxel_size_;

public:		// Vars
	ros::Publisher test_pub1_;
	ros::Publisher test_pub2_;

	ros::Subscriber raw_pc_sub_;

private: 	// Functions
	pointCloud::Ptr threasholdAxis_(pointCloud::Ptr pc, const std::string axis, float min, float max);

	pointCloud::Ptr voxelDownSample_(pointCloud::Ptr pc, float voxel_size);

	std::vector<pcl::PointIndices> euclideanSegment_(pointCloud::Ptr pc, float max_distance, int min_cluster_size, int max_cluster_size);

public:		// Funcitons
	object_extractor();

	void CloudCb_(const point_msg::ConstPtr& pc);


};

object_extractor::object_extractor() :
		min_height_(0),
		max_height_(5)
{

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	raw_pc_sub_ = nh.subscribe<point_msg>("/camera/depth/points", 10, &object_extractor::CloudCb_, this);

	test_pub1_ = nh.advertise<point_msg>(nh.resolveName("test_pc1"), 10);
	test_pub2_ = nh.advertise<point_msg>(nh.resolveName("test_pc2"), 10);

	//Get some ros params
	std::string name = private_nh.resolveName("min_pc_height");

	private_nh.param<double>(name.c_str(), min_height_, -0.5);
	ROS_INFO("Param %s value %f", name.c_str(), min_height_);

	name = private_nh.resolveName("max_pc_height");
	private_nh.param<double>(name.c_str(), max_height_, 0.05);
	ROS_INFO("Param %s value %f", name.c_str(), max_height_);

	name = private_nh.resolveName("euclidean_cluster_tolerance");					//euclidean_cluster_tolerance
	private_nh.param<double>(name.c_str(), cluster_tolerance_, 0.05);
	ROS_INFO("Param %s value %f", name.c_str(), cluster_tolerance_);

	name = private_nh.resolveName("euclidean_min_cluster_size");					//euclidean_min_cluster_size
	private_nh.param<int>(name.c_str(), min_cluster_size_, 5);
	ROS_INFO("Param %s value %i", name.c_str(), min_cluster_size_);

	name = private_nh.resolveName("euclidean_max_cluster_size");					//euclidean_max_cluster_size
	private_nh.param<int>(name.c_str(), max_cluster_size_, 0xFFFFF);
	ROS_INFO("Param %s value %i", name.c_str(), max_cluster_size_);

	name = private_nh.resolveName("voxel_size");									//Size of a voxel
	private_nh.param<double>(name.c_str(), voxel_size_, 0.01);
	ROS_INFO("Param %s value %i", name.c_str(), max_cluster_size_);
}

// Threashold axis
object_extractor::pointCloud::Ptr object_extractor::threasholdAxis_(pointCloud::Ptr pc, const std::string axis, float min, float max)
{
	ROS_INFO("None axis threasholded pc has %i data points", pc->width * pc->height);
	// Get rid of points to low and points that are to high
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<point> pass;
	pass.setInputCloud (pc);
	pass.setFilterFieldName ("y");		// TODO: use axis instead
	pass.setFilterLimits (min, max);
	pointCloud::Ptr out_pc(new pointCloud);
	pass.filter (*out_pc);
	ROS_INFO("Axis (%s) threasholded pc has %i data points", axis.c_str(), out_pc->width * out_pc->height);
	return out_pc;
}

// Down sample with voxels
object_extractor::pointCloud::Ptr object_extractor::voxelDownSample_(pointCloud::Ptr pc, float voxel_size)
{
	ROS_INFO("Before %f cube voxel grid downsample point cloud has %i data points", voxel_size, pc->width * pc->height);
	pcl::VoxelGrid<point> vox;
	vox.setInputCloud(pc);
	vox.setLeafSize(voxel_size, voxel_size, voxel_size);
	pointCloud::Ptr vox_pc(new pointCloud);
	vox.filter(*vox_pc);
	ROS_INFO("After %f cube voxel grid downsample point cloud has %i data points", voxel_size, vox_pc->width * vox_pc->height);
	return vox_pc;
}

// Euclidean segmentation
// TODO: Should the vector be returned as a pointer???
std::vector<pcl::PointIndices> object_extractor::euclideanSegment_(pointCloud::Ptr pc, float max_distance, int min_cluster_size, int max_cluster_size)
{
	//Create KdTree for search method (see pcl documentation for more info on KdTrees)
	//And give it input
	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
	tree->setInputCloud(pc);

	//Create a vector of vectors which represent the indices of each cluster
	std::vector<pcl::PointIndices> cluster_indices;

	//Set up the Euclidean Extractor
	pcl::EuclideanClusterExtraction<point> extractor;
	extractor.setClusterTolerance(max_distance);			//Points must be within cluster_tolerance meters of eachother
	extractor.setMinClusterSize(min_cluster_size);			//Min number of points in cluster
	extractor.setMaxClusterSize(max_cluster_size);			//Max number of points in cluster
	extractor.setSearchMethod(tree);						//search method
	extractor.setInputCloud(pc);							//Input cloud
	
	//Perform extraction
	extractor.extract(cluster_indices);
	//pcl::extractEuclideanClusters(*pc, cluster_indices, )
	ROS_INFO("Number of clusters is %i", (int)cluster_indices.size());
	return cluster_indices;

}

void object_extractor::CloudCb_(const point_msg::ConstPtr& pc)
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
	// Down sampeling
	pointCloud::Ptr height_filtered_pc = threasholdAxis_(raw_pc, "y", min_height_, max_height_);

	// Debug output
	point_msg::Ptr out_msg(new point_msg);
	pcl::toROSMsg(*height_filtered_pc, *out_msg);
	test_pub1_.publish(out_msg);

	//VoxelGrid downsample
	pointCloud::Ptr vox_pc = voxelDownSample_(height_filtered_pc, voxel_size_);
	
	// Euclideian segmentation
	std::vector<pcl::PointIndices> cluster_indices = euclideanSegment_(vox_pc, cluster_tolerance_, min_cluster_size_, max_cluster_size_);

	//Publish the segmented cloud
	point_msg::Ptr out_pc(new sensor_msgs::PointCloud2);							//Create the ros msg

	for(std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin();	//Iterate through clusters
			cluster_it != cluster_indices.end(); ++cluster_it)
	{

		pcl::PointCloud<point>::Ptr cluster_pc(new pcl::PointCloud<point>);						//Make a new point cloud

		pcl::copyPointCloud(*vox_pc, cluster_it->indices, *cluster_pc);							//Copy new point cloud

		pcl::toROSMsg(*cluster_pc, *out_pc);													//Fill the msg
		out_pc->header = pc->header;															//Change the header
		/*out_pc->header.frame_id = pc->header.frame_id;										//Set frame id
		out_pc->header.stamp = pc->header.stamp;*/												//Set time stamp

		// Test for spheres
		test_pub2_.publish(out_pc);																//Publish the msgs

		ROS_INFO("This euclidean segment has %i data points", cluster_pc->width * cluster_pc->height);
	}

	ROS_INFO("Starting Sphere extraction");
	// Estimate point normals
	// Make kd tree
	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
	pcl::NormalEstimation<point, pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(vox_pc);
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
	seg.setInputCloud(vox_pc);
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

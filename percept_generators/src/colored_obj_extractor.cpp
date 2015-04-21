#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <percept_generators/object.h>
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

#include <cmath>

#include <stdio.h>


class object_extractor
{
private:	// Type defs
	typedef pcl::PointXYZ point;
	typedef pcl::PointCloud<point> pointCloud;
	typedef sensor_msgs::PointCloud2 point_msg;
	typedef Eigen::Matrix<float, 4, 1> vector4;

private:	// Vars
	double min_height_;
	double max_height_;
	double cluster_tolerance_;			//Max distance in meters between points
	int min_cluster_size_;		//Minimum number of points in a cluster
	int max_cluster_size_;		//Maximum number of points in a cluster
	double voxel_size_;
	ros::Publisher sphere_pub_;
	ros::Publisher cylinder_pub_;
	ros::Publisher scene_pub_;
	ros::Subscriber raw_pc_sub_;
	ros::Publisher object_pub_;
	double cylinder_axis_tolerance_;		// The maximum number of +-degrees from vertical allowed before we don't consider it a cylinder

public:		// Vars


private: 	// Functions
	pointCloud::Ptr threasholdAxis_(pointCloud::Ptr pc, const std::string axis, float min, float max);

	pointCloud::Ptr voxelDownSample_(pointCloud::Ptr pc, float voxel_size);

	std::vector<pcl::PointIndices> euclideanSegment_(pointCloud::Ptr pc, float max_distance, int min_cluster_size, int max_cluster_size);

	void fitShape_(pointCloud::Ptr pc, percept_generators::object& obj);

	void StampAndPublishObject(percept_generators::object);

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

	raw_pc_sub_ = nh.subscribe<point_msg>("/hardware_interface/camera/depth/points", 10, &object_extractor::CloudCb_, this);

	sphere_pub_ = nh.advertise<point_msg>(nh.resolveName("sphere_pc"), 10);
	cylinder_pub_ = nh.advertise<point_msg>(nh.resolveName("cylinder_pc"), 10);
	scene_pub_ = nh.advertise<point_msg>(nh.resolveName("scene_pc"), 10);
	object_pub_ = nh.advertise<percept_generators::object>(nh.resolveName("objects"), 10);

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
	ROS_INFO("Param %s value %f", name.c_str(), voxel_size_);

	name = private_nh.resolveName("cylinder_axis_tolerance");						// The maximum number of +-degrees from vertical allowed before we don't consider it a cylinder
	private_nh.param<double>(name.c_str(), cylinder_axis_tolerance_, 10);
	ROS_INFO("Param %s value %f", name.c_str(), cylinder_axis_tolerance_);
	cylinder_axis_tolerance_ = cos(cylinder_axis_tolerance_ * M_PI / 180);			// One time conversion

}

// Threashold axis
object_extractor::pointCloud::Ptr object_extractor::threasholdAxis_(pointCloud::Ptr pc, const std::string axis, float min, float max)
{
	//ROS_INFO("Pre-axis threasholded pc has %i data points", pc->width * pc->height);
	// Get rid of points to low and points that are to high
	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<point> pass;
	pass.setInputCloud (pc);
	pass.setFilterFieldName ("y");		// TODO: use axis instead
	pass.setFilterLimits (min, max);
	pointCloud::Ptr out_pc(new pointCloud);
	pass.filter (*out_pc);
	//ROS_INFO("Axis (%s) threasholded pc has %i data points", axis.c_str(), out_pc->width * out_pc->height);
	return out_pc;
}

// Down sample with voxels
object_extractor::pointCloud::Ptr object_extractor::voxelDownSample_(pointCloud::Ptr pc, float voxel_size)
{
	//ROS_INFO("Before %f cube voxel grid downsample point cloud has %i data points", voxel_size, pc->width * pc->height);
	pcl::VoxelGrid<point> vox;
	vox.setInputCloud(pc);
	vox.setLeafSize(voxel_size, voxel_size, voxel_size);
	pointCloud::Ptr vox_pc(new pointCloud);
	vox.filter(*vox_pc);
	//ROS_INFO("After %f cube voxel grid downsample point cloud has %i data points", voxel_size, vox_pc->width * vox_pc->height);
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
	//ROS_INFO("Number of clusters is %i", (int)cluster_indices.size());
	return cluster_indices;

}

void object_extractor::fitShape_(pointCloud::Ptr pc, percept_generators::object& obj)
{
	//ROS_INFO("Starting shape extraction");
	int size_of_input_pc = pc->width * pc->height;
	// Experimentally determined
	//	All of the objects I care about are less than 1000 data points
	if(size_of_input_pc > 1000)
	{
		obj.type = percept_generators::object::UNKOWN;		// Set model to some undefined value
		vector4 centroid;
		pcl::compute3DCentroid(*pc, centroid);
		obj.centroid.x = centroid(0);
		obj.centroid.y = 0;
		obj.centroid.z = centroid(2);
		return;
	}

	// Estimate point normals
	// Make kd tree
	pcl::search::KdTree<point>::Ptr tree(new pcl::search::KdTree<point>);
	pcl::NormalEstimation<point, pcl::Normal> ne;
	ne.setSearchMethod(tree);
	ne.setInputCloud(pc);
	ne.setKSearch(50);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*normals);

	// Set up segmenter for spheres
	pcl::SACSegmentationFromNormals<point, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_SPHERE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(50);
	seg.setDistanceThreshold(0.05);
	seg.setRadiusLimits(0.05, 0.20);
	seg.setInputCloud(pc);
	seg.setInputNormals(normals);

	// Segment out sphere
	pcl::PointIndices sphere_inliers;
	pcl::ModelCoefficients sphere_coeffs;
	seg.segment(sphere_inliers, sphere_coeffs);

	float sphere_fit = 1.0 * sphere_inliers.indices.size() / size_of_input_pc;

	// Set up segmenter for cylinders
	seg.setModelType(pcl::SACMODEL_CYLINDER);

	// Segment out sphere
	pcl::PointIndices cylinder_inliers;
	pcl::ModelCoefficients cylinder_coeffs;
	seg.segment(cylinder_inliers, cylinder_coeffs);

	float cylinder_fit = 1.0 * cylinder_inliers.indices.size() / size_of_input_pc;

	// Determine if sphere cylinder or neither
	if(cylinder_fit >= 0.9)
	{
		// Check the central axis of the cylinder model is on the y axis
		// Cylinder coeffs px, py, pz, ax, ay, az, r
		// 	with p being point on axis, a being axis vector, and r being radius
		// Since the cylinder axis is a unit vector, doting the y_axis and cylinder axis 
		// Should be either about 1 or about -1
		// To avoid acos we just use a +- cylinder_axis_tolerance degrees pre calculated limits ()
		Eigen::Vector3f y_axis(0, 1, 0);
		Eigen::Vector3f cylinder_axis(cylinder_coeffs.values[3],
						cylinder_coeffs.values[4], cylinder_coeffs.values[5]);
		float dp = y_axis.dot(cylinder_axis);
		if(dp > cylinder_axis_tolerance_ || dp < -cylinder_axis_tolerance_)
		{

			vector4 centroid;
			pcl::compute3DCentroid(*pc, cylinder_inliers, centroid);
			obj.centroid.x = centroid(0);
			obj.centroid.y = 0;
			obj.centroid.z = centroid(2);

			obj.type = percept_generators::object::CYLINDER;
			//ROS_INFO("CYLINDER:\nFit: %f%%\nCloud size: %i\nCoeffs: [%f, %f, %f, %f, %f, %f, %f]\n", 
			//	cylinder_fit * 100,
			//	size_of_input_pc,
			//	cylinder_coeffs.values[0], cylinder_coeffs.values[1], cylinder_coeffs.values[2], cylinder_coeffs.values[3],
			//		cylinder_coeffs.values[4], cylinder_coeffs.values[5], cylinder_coeffs.values[6]);

			//Publish the segmented cloud
			point_msg::Ptr out_pc(new sensor_msgs::PointCloud2);									//Create the ros msg
			pointCloud::Ptr shape_pc(new pointCloud);
			pcl::copyPointCloud(*pc, cylinder_inliers, *shape_pc);
			pcl::toROSMsg(*pc, *out_pc);													//Fill the msg
			out_pc->header.frame_id = "/camera_depth_optical_frame";
			out_pc->header.stamp = ros::Time::now();
			cylinder_pub_.publish(out_pc);																//Publish the msgs
			return;
		}
	}
	else if(sphere_fit >= 0.9)
	{
		vector4 centroid;
		pcl::compute3DCentroid(*pc, cylinder_inliers, centroid);
		obj.centroid.x = centroid(0);
		obj.centroid.y = 0;
		obj.centroid.z = centroid(2);

		obj.type = percept_generators::object::SPHERE;
		//ROS_INFO("SPHERE:\nFit: %f%%\nCloud size: %i\nCoeffs: [%f, %f, %f, %f]\n", 
		//		sphere_fit * 100,
		//		size_of_input_pc,
		//		sphere_coeffs.values[0], sphere_coeffs.values[1], sphere_coeffs.values[2], sphere_coeffs.values[3]);

		//Publish the segmented cloud
		point_msg::Ptr out_pc(new sensor_msgs::PointCloud2);									//Create the ros msg
		pointCloud::Ptr shape_pc(new pointCloud);
		pcl::copyPointCloud(*pc, sphere_inliers, *shape_pc);
		pcl::toROSMsg(*pc, *out_pc);															//Fill the msg
		out_pc->header.frame_id = "/camera_depth_optical_frame";
		out_pc->header.stamp = ros::Time::now();
		sphere_pub_.publish(out_pc);																//Publish the msgs
		return;
	}
	else
	{
		obj.type = percept_generators::object::UNKOWN;		// Set model to some undefined value
		vector4 centroid;
		pcl::compute3DCentroid(*pc, centroid);
		obj.centroid.x = centroid(0);
		obj.centroid.y = 0;
		obj.centroid.z = centroid(2);
		return;
	}

}

// Publishes a object (esentialy this is just here as a convinence function)
//		By the xz cordinates of the centroid (The kinect defines y as up down, x left right, and z as depth)
void object_extractor::StampAndPublishObject(percept_generators::object obj)
{
	obj.header.frame_id = "/camera_depth_optical_frame";
	obj.header.stamp = ros::Time::now();

	object_pub_.publish(obj);
}


void object_extractor::CloudCb_(const point_msg::ConstPtr& pc)
{
	// Convert from ros msg
	pointCloud::Ptr raw_pc(new pointCloud);
	pcl::fromROSMsg(*pc, *raw_pc);

	// Down sampeling
	pointCloud::Ptr height_filtered_pc = threasholdAxis_(raw_pc, "y", min_height_, max_height_);

	// VoxelGrid downsample
	pointCloud::Ptr vox_pc = voxelDownSample_(height_filtered_pc, voxel_size_);

	// Debug output scene
	point_msg scene;
	pcl::toROSMsg(*vox_pc, scene);
	scene.header.frame_id = "/camera_depth_optical_frame";
	scene.header.stamp = ros::Time::now();
	scene_pub_.publish(scene);
	
	// Euclideian segmentation
	std::vector<pcl::PointIndices> cluster_indices = euclideanSegment_(vox_pc, cluster_tolerance_, min_cluster_size_, max_cluster_size_);


	for(std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin();	//Iterate through clusters
			cluster_it != cluster_indices.end(); ++cluster_it)
	{
		pointCloud::Ptr cluster_pc(new pointCloud);												//Make a new point cloud
		//ROS_INFO("This euclidean segment has %i data points", cluster_pc->width * cluster_pc->height);

		pcl::copyPointCloud(*vox_pc, cluster_it->indices, *cluster_pc);							//Copy new point cloud

		percept_generators::object obj;															//Fit a shape to the data
		fitShape_(cluster_pc, obj);									

		StampAndPublishObject(obj);				
	}

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

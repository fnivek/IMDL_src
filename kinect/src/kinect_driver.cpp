#include <ros/ros.h>
#include <ros/rate.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>


/************************
 * 			Main		*
 ************************/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "kinect_driver");
	ros::NodeHandle nh;
	ros::Rate update_rate(100);

	while(ros::ok())
	{
		update_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}

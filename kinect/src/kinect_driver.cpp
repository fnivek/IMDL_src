#include <ros/ros.h>
#include <ros/rate.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/openni_grabber.h>

class temp
{
public:
	pcl::Grabber* interface_;

	ros::Publisher pc_pub_;

	ros::Duration delta_;
	ros::Time last_time_;

	void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	{
		// Convert to ros msg
		sensor_msgs::PointCloud2::Ptr out_pc(new sensor_msgs::PointCloud2);
		pcl::toROSMsg(*cloud, *out_pc);

		// Fill out header
		out_pc->header.frame_id = "base_link";

		// Publish raw point cloud
		pc_pub_.publish(out_pc);

		delta_ = ros::Time::now() - last_time_;
		last_time_ = ros::Time::now();

		ROS_INFO("Delta: %f", delta_.toSec());
	}

	void init_(ros::NodeHandle& nh)
	{
		pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("raw_pc", 10);
		last_time_ = ros::Time::now();

		// Test openni
		while(ros::ok())
		{
			if(interface_ == NULL)
			{
				try
				{
					interface_ = new pcl::OpenNIGrabber();
				}
				catch(const openni_wrapper::OpenNIException& e)
				{
					ROS_ERROR("OpenNIException occurred while connecting to the kinect \n\t%s", e.what());
				}
				catch(const pcl::IOException& e)
				{
					ROS_ERROR("IOException occurred while connecting to the kinect \n\t%s", e.what());
				};
			}
			else
			{
				break;
			}
		}


		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> callback =
				 boost::bind (&temp::cloud_cb_, this, _1);

		interface_->registerCallback (callback);

		interface_->start ();
	}

	void stop_()
	{
		if(interface_ != NULL)
		{
			interface_->stop();
		}

	}

	temp():
		interface_(NULL)
	{

	}
};


/************************
 * 			Main		*
 ************************/
int main(int argc, char** argv)
{
	//Initialize ROS
	ros::init(argc, argv, "kinect_driver");
	ros::NodeHandle nh;
	ros::Rate update_rate(100);

	temp test;
	test.init_(nh);

	while(ros::ok())
	{
		update_rate.sleep();
		ros::spinOnce();
	}

	test.stop_();

	return 0;
}

#include <ros/ros.h>
#include <string>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;

class LaserscanMerger
{
public:
    LaserscanMerger();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
    void setAngleLimits(const sensor_msgs::LaserScan::ConstPtr& msg, sensor_msgs::LaserScan& filtered_scan);
    void pointcloud_to_laserscan(pcl::PCLPointCloud2 *merged_cloud);
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);

private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Publisher laser_scan_publisher_;
    vector<ros::Subscriber> scan_subscribers;
    vector<bool> clouds_modified;

    vector<pcl::PCLPointCloud2> clouds;

    void laserscan_topic_parser();

    double limit_angle_min;
    double limit_angle_max;

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;
    double intensity_min;
    bool set_inf_to_the_points_exceed_range_max;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;
};

void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

void LaserscanMerger::laserscan_topic_parser()
{
    istringstream iss(laserscan_topics);
	for(string input_topic; iss>>input_topic;){
    	scan_subscribers.push_back(node_.subscribe<sensor_msgs::LaserScan>(
        	input_topic, 1, boost::bind(&LaserscanMerger::scanCallback, this, _1, input_topic)));
        
    }
    if (scan_subscribers.empty())
        ROS_WARN("Not subscribed to any topic.");

    ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
    clouds_modified = vector<bool>(scan_subscribers.size(), false);
    clouds.resize(scan_subscribers.size());
}

LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

    nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
    nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
    nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
    nh.param<std::string>("laserscan_topics", laserscan_topics, "");
    nh.param("limit_angle_min", limit_angle_min, -2.0933);
    nh.param("limit_angle_max", limit_angle_max, 2.0933);
    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.45);
    nh.param("range_max", range_max, 25.0);
    nh.param("intensity_min", intensity_min, 1000.0);
    nh.param("set_inf_to_the_points_exceed_range_max", set_inf_to_the_points_exceed_range_max, false);

    this->laserscan_topic_parser();

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);

}

void LaserscanMerger::setAngleLimits(const sensor_msgs::LaserScan::ConstPtr& scan, sensor_msgs::LaserScan& filtered_scan)
{
	filtered_scan.header = scan->header;
	filtered_scan.angle_increment = scan->angle_increment;
	filtered_scan.time_increment = scan->time_increment;
	filtered_scan.scan_time = scan->scan_time;
	filtered_scan.range_min = scan->range_min;
	filtered_scan.range_max = scan->range_max;
	filtered_scan.angle_min = this->limit_angle_min;
	filtered_scan.angle_max = this->limit_angle_max;
    bool has_intensity = !scan->intensities.empty();

    if (!has_intensity)
    {
		ROS_ERROR("Intensities array is empty in the LaserScan message.");
    }

	for (unsigned int i = 0; i < scan->ranges.size(); ++i)
	{
		float angle = scan->angle_min + i * scan->angle_increment;
		if (angle >= this->limit_angle_min && angle <= this->limit_angle_max)
		{
			filtered_scan.ranges.push_back(scan->ranges[i]);
			if (has_intensity)
			{
				filtered_scan.intensities.push_back(scan->intensities[i]);
			}
		}
	}
}

void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic)
{
	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;
	sensor_msgs::LaserScan filtered_scan;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
	setAngleLimits(scan, filtered_scan);
	projector_.transformLaserScanToPointCloud(filtered_scan.header.frame_id, filtered_scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
	try
	{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	for(int i=0; i<scan_subscribers.size(); ++i)
	{
		if(topic.compare(scan_subscribers[i].getTopic()) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}else if(scan_subscribers[i].getNumPublishers() == 0){
			clouds_modified[i] = true;
		}
	}	

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
	
		point_cloud_publisher_.publish(merged_cloud);

		pointcloud_to_laserscan(&merged_cloud);
	}
}

void LaserscanMerger::pointcloud_to_laserscan(pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
	output->header.stamp = ros::Time::now();
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;
	std::vector<float> intensities;
	
	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	if (set_inf_to_the_points_exceed_range_max)
	{
		output->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
	}
	else
	{
		output->ranges.assign(ranges_size, output->range_max + 1.0);
	}

	for (size_t i = 0; i < merged_cloud->data.size(); i += merged_cloud->point_step)
	{
		float x, y, z;
		float intensity_value;
		memcpy(&x, &merged_cloud->data[i], sizeof(float));
		memcpy(&y, &merged_cloud->data[i + sizeof(float)], sizeof(float));
		memcpy(&z, &merged_cloud->data[i + 2 * sizeof(float)], sizeof(float));
		memcpy(&intensity_value, &merged_cloud->data[i + 3 * sizeof(float)], sizeof(float));
		
		if (std::isnan(x) || std::isnan(y) || std::isnan(z))
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_)
		{
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double range_max_sq_ = output->range_max * output->range_max;
		if (range_max_sq_ <= range_sq)
		{
			ROS_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range_sq, range_max_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}

		int index = (angle - output->angle_min) / output->angle_increment;

		if (intensity_value > intensity_min)
		{
			if (output->ranges[index] * output->ranges[index] > range_sq)
			{
				output->ranges[index] = sqrt(range_sq);
				intensities.resize(ranges_size, 0.0f);
			}

			if (intensities[index] < intensity_value)
			{
				intensities[index] = intensity_value;
			}
		}
	}

	output->intensities.resize(intensities.size());
	output->intensities = intensities;

	laser_scan_publisher_.publish(output);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_multi_merger");
	LaserscanMerger _laser_merger;

	dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
	dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

	f = boost::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}

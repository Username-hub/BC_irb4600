#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;
visualization_msgs::Marker point_cloud_marker;
ros::Publisher marker_publisher;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    // Do data processing here...
    output = *input;
    point_cloud_marker.points.clear();
    ROS_DEBUG_STREAM("Width " <<  output.width << " heigt " << output.height);

    for(int i = 0; i < output.width; i++)
    {
        for(int j = 0; j < output.height; i++)
        {
            geometry_msgs::Point new_point;
            new_point.x = i;
            new_point.y = j;
            new_point.z = output.data[i,j];
            point_cloud_marker.points.push_back(new_point);
        }
    }
    marker_publisher.publish(point_cloud_marker);
    // Publish the data.
    //pub.publish (output);

}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "start_volumetric_bnv");
    ros::NodeHandle n;
    ros::Rate r(1);
    // /kinect/depth/points
    ros::Subscriber depth_camera_subscriber = n.subscribe("kinect/depth/points",1, cloud_cb);

    //pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);
        marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

        point_cloud_marker.header.frame_id = "base_link";
        point_cloud_marker.header.stamp = ros::Time();
        point_cloud_marker.ns = "my_namespace";
        point_cloud_marker.id = 0;
        point_cloud_marker.type = visualization_msgs::Marker::CUBE_LIST;
        point_cloud_marker.action = visualization_msgs::Marker::ADD;



        point_cloud_marker.pose.position.x = 0;
        point_cloud_marker.pose.position.y = 0;
        point_cloud_marker.pose.position.z = 0;
        point_cloud_marker.pose.orientation.x = 0.0;
        point_cloud_marker.pose.orientation.y = 0.0;
        point_cloud_marker.pose.orientation.z = 0.0;
        point_cloud_marker.pose.orientation.w = 1.0;
        point_cloud_marker.scale.x = 0.1;
        point_cloud_marker.scale.y = 0.1;
        point_cloud_marker.scale.z = 0.1;
        point_cloud_marker.color.a = 1.0; // Don't forget to set the alpha!
        point_cloud_marker.color.r = 0.0;
        point_cloud_marker.color.g = 1.0;
        point_cloud_marker.color.b = 0.0;



        marker_publisher.publish(point_cloud_marker);
        ros::spin();

}


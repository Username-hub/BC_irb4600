#include <math.h>

#include <map>
#include <set>
#include <list>
#include <queue>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>


namespace enc = sensor_msgs::image_encodings;

ros::Publisher pub;
visualization_msgs::Marker point_cloud_marker;
ros::Publisher marker_publisher;


void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
{
// get width and height of 2D point cloud data
    int width = pCloud.width;
    int height = pCloud.height;

// Convert from u (column / width), v (row/height) to position in array
// where X,Y,Z data starts
    int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

// compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;

}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{

    sensor_msgs::PointCloud2 output;

    output = *cloud;

    point_cloud_marker.points.clear();

    for(int i = 0; i < output.width; i+=10)
    {
        for(int j = 0; j < output.height; j+=10)
        {
            geometry_msgs::Point new_point ;
            pixelTo3DPoint(output, i, j, new_point);
            point_cloud_marker.points.push_back(new_point);
        }
        //std::cout << i << std::endl;
    }

    std::cout << "marker publish" << std::endl;
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

        point_cloud_marker.header.frame_id = "link_depthtf";
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
        point_cloud_marker.scale.x = 0.01;
        point_cloud_marker.scale.y = 0.01;
        point_cloud_marker.scale.z = 0.01;
        point_cloud_marker.color.a = 1.0; // Don't forget to set the alpha!
        point_cloud_marker.color.r = 0.0;
        point_cloud_marker.color.g = 1.0;
        point_cloud_marker.color.b = 0.0;



        marker_publisher.publish(point_cloud_marker);
        ros::spin();

}


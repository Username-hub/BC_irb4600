#include <math.h>

#include <map>
#include <set>
#include <list>
#include <queue>
#include <chrono>
#include <vector>

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


#ifndef SOURCE_FRAME
#define SOURCE_FRAME "base_link"
#endif
#ifndef TARGET_FRAME
#define TARGET_FRAME "link_depthtf"
#endif
#define PI 3.14159265

namespace enc = sensor_msgs::image_encodings;

ros::Publisher pub;
visualization_msgs::Marker point_cloud_marker;
visualization_msgs::Marker position_marker;
ros::Publisher marker_publisher;
ros::Publisher position_publisher;

struct map {
    std::vector<geometry_msgs::Point> maped_points;
}map_points_list,current_scan;

enum StateOfRobot{ initial_scan, generate_candidate_view, evaluating_view, move_to_pose, making_scan, add_points} robot_state;

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
void UpdateMarker()
{
    point_cloud_marker.points.clear();
    for(int i = 0; i < map_points_list.maped_points.size(); i++)
    {
        point_cloud_marker.points.push_back(map_points_list.maped_points.at(i));
    }
}

class candidateCameraView
{
public:
    float x;
    float y;
    float z;

    candidateCameraView(float r, float s, float t, float xOffset, float yOffset, float zOffset )
    {
        x = r * cos(s) * sin(t) + xOffset;
        y = r * sin(s) * sin(t) + yOffset;
        z = r * cos(t) + zOffset;
    }

    geometry_msgs::Point GetMessagePoint()
    {
        geometry_msgs::Point result;
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    }
};
void generateCandiadateViews()
{
    std::cout<<"Candidate generation START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    float r = 5.0;
    float xOffset = 1.0, yOffset = 0.0 , zOffset = 1.0;
    float verticalStep = 30.0, horizontalStep = 30.0;
    std::vector<candidateCameraView> views;

    for(float s = 0; s < 360; s+= verticalStep)
    {
        for(float t = 0; t < 360; t += horizontalStep)
        {
            candidateCameraView CAV(r, s * PI/180, t * PI/180,xOffset, yOffset, zOffset);
            views.push_back(CAV);
            position_marker.points.push_back(CAV.GetMessagePoint());
        }
    }
    position_publisher.publish(position_marker);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Candidate generation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}

void initialScan()
{
    std::cout<<"Initial Scan START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(int i = 0; i < current_scan.maped_points.size(); i++)
    {
        map_points_list.maped_points.push_back(current_scan.maped_points.at(i));
    }
    UpdateMarker();
    robot_state = generate_candidate_view;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Initial Scan END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    generateCandiadateViews();
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{

    sensor_msgs::PointCloud2 output;

    output = *cloud;

    tf::Stamped<tf::Point> stamped_point;
    stamped_point.frame_id_ = TARGET_FRAME;

    tf::TransformListener tf_listner;

    current_scan.maped_points.clear();
    for(int i = 0; i < output.width; i+=10)
    {
        for(int j = 0; j < output.height; j+=10)
        {
            //Declare point to write relative to camera position
            geometry_msgs::Point new_point ;
            //Get point relative to camera position
            pixelTo3DPoint(output, i, j, new_point);
            //Create point used in translation to different frame
            tf::Point tf_point(new_point.x,new_point.y,new_point.z);
            //Set point
            stamped_point.setData(tf_point);

            //Result point
            tf::Stamped<tf::Point> stamped_point2;
            stamped_point2.setData(tf_point);
            //Set frame
            stamped_point2.frame_id_ = SOURCE_FRAME;
            //Point transformation
            tf_listner.waitForTransform(SOURCE_FRAME,TARGET_FRAME,ros::Time(0),ros::Duration(5.0));
            tf_listner.transformPoint(SOURCE_FRAME,stamped_point,stamped_point2);

            geometry_msgs::Point finalPoint;
            finalPoint.x = stamped_point2.x();
            finalPoint.y = stamped_point2.y();
            finalPoint.z = stamped_point2.z();
            current_scan.maped_points.push_back(finalPoint);
            //point_cloud_marker.points.push_back(finalPoint);

        }

    }

    std::cout << "marker publish" << std::endl;
    marker_publisher.publish(point_cloud_marker);
    if(robot_state == initial_scan)
    {
        initialScan();
    }

    // Publish the data.
    //pub.publish (output);
}

void initCloudMarker()
{
    point_cloud_marker.header.frame_id = SOURCE_FRAME;
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
}

void initLineMarker()
{
    position_marker.header.frame_id = SOURCE_FRAME;
    position_marker.header.stamp = ros::Time();
    position_marker.ns = "my_namespace";
    position_marker.id = 0;
    position_marker.type = visualization_msgs::Marker::LINE_LIST;
    position_marker.action = visualization_msgs::Marker::ADD;

    position_marker.pose.position.x = 0;
    position_marker.pose.position.y = 0;
    position_marker.pose.position.z = 0;
    position_marker.pose.orientation.x = 0.0;
    position_marker.pose.orientation.y = 0.0;
    position_marker.pose.orientation.z = 0.0;
    position_marker.pose.orientation.w = 1.0;
    position_marker.scale.x = 0.01;
    position_marker.scale.y = 0.01;
    position_marker.scale.z = 0.01;
    position_marker.color.a = 1.0; // Don't forget to set the alpha!
    position_marker.color.r = 0.0;
    position_marker.color.g = 0.0;
    position_marker.color.b = 1.0;
}


int main( int argc, char** argv )
{
    robot_state = initial_scan;
    ros::init(argc, argv, "start_volumetric_bnv");
    ros::NodeHandle n;
    ros::Rate r(1);
    // /kinect/depth/points
    ros::Subscriber depth_camera_subscriber = n.subscribe("kinect/depth/points",1, cloud_cb);

    marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    initCloudMarker();

    position_publisher = n.advertise<visualization_msgs::Marker>("position_marker", 0);
    initLineMarker();

    marker_publisher.publish(point_cloud_marker);
    ros::spin();

}


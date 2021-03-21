#include <math.h>

#include <map>
#include <set>
#include <list>
#include <queue>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include "point_translation.h"
#include "markers_init.h"
#include "ray_box_collider.h"

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>

#ifndef SOURCE_FRAME
#define SOURCE_FRAME "base_link"
#endif
#ifndef TARGET_FRAME
#define TARGET_FRAME "link_depthtf"
#endif
#define PI 3.14159265

//namespace enc = sensor_msgs::image_encodings;

ros::Publisher pub;
visualization_msgs::Marker point_cloud_marker;
visualization_msgs::Marker position_marker;
ros::Publisher marker_publisher;
ros::Publisher position_publisher;

struct map {
    std::vector<geometry_msgs::Point> maped_points;
}map_points_list,current_scan;

enum StateOfRobot{ initial_scan, generate_candidate_view, evaluating_view, move_to_pose, making_scan, add_points} robot_state;

void UpdateMarker()
{
    point_cloud_marker.points.clear();
    for(int i = 0; i < map_points_list.maped_points.size(); i++)
    {
        point_cloud_marker.points.push_back(map_points_list.maped_points.at(i));
    }
    marker_publisher.publish(point_cloud_marker);
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
    //write scan to map
    if(robot_state == initial_scan) {
        sensor_msgs::PointCloud2 output;

        output = *cloud;

        tf::Stamped<tf::Point> stamped_point;
        stamped_point.frame_id_ = TARGET_FRAME;

        tf::TransformListener tf_listner;

        current_scan.maped_points.clear();
        for (int i = 0; i < output.width; i += 10) {
            for (int j = 0; j < output.height; j += 10) {
                //Declare point to write relative to camera position
                geometry_msgs::Point new_point;
                //Get point relative to camera position
                pixelTo3DPoint(output, i, j, new_point);
                //Create point used in translation to different frame
                tf::Point tf_point(new_point.x, new_point.y, new_point.z);
                //Set point
                stamped_point.setData(tf_point);

                //Result point
                tf::Stamped<tf::Point> stamped_point2;
                stamped_point2.setData(tf_point);
                //Set frame
                stamped_point2.frame_id_ = SOURCE_FRAME;
                //Point transformation
                tf_listner.waitForTransform(SOURCE_FRAME, TARGET_FRAME, ros::Time(0), ros::Duration(5.0));
                tf_listner.transformPoint(SOURCE_FRAME, stamped_point, stamped_point2);

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
    }
    //Call initial scan function
    if(robot_state == initial_scan)
    {
        initialScan();
    }

    // Publish the data.
    //pub.publish (output);
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
    initCloudMarker(point_cloud_marker);

    position_publisher = n.advertise<visualization_msgs::Marker>("position_marker", 0);
    initLineMarker(position_marker);

    marker_publisher.publish(point_cloud_marker);
    ros::spin();

}


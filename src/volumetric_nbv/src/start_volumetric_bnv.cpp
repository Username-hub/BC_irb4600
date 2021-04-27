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
#include <moveit/move_group_interface/move_group_interface.h>
#include "point_translation.h"
#include "markers_init.h"
#include "ray_box_collider.h"
#include "octomap.h"
#include "candidateCameraView.h"
#include "moveControl.h"

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
Vec3f center(1.5,0,0.5);


Octomap octomap_local(Vec3f(3,3,3),Vec3f(-3,-3,0),0.1f);
struct map {
    std::vector<geometry_msgs::Point> maped_points;
}map_points_list,current_scan;

enum StateOfRobot{ initial_scan, generate_candidate_view, evaluating_view, move_to_pose, wait_move, making_scan, add_points} robot_state_scan;

void UpdateMarker()
{
    point_cloud_marker.points.clear();
    for(int i = 0; i < map_points_list.maped_points.size(); i++)
    {
        point_cloud_marker.points.push_back(map_points_list.maped_points.at(i));
    }
    marker_publisher.publish(point_cloud_marker);

}


geometry_msgs::Pose target_pose;
void MoveToPose()
{
    setRotation(target_pose,center);
    MoveControlClass moveControlClass;
    moveControlClass.MoveToPoint(target_pose);
    robot_state_scan = making_scan;
}
void evaluateCameraViews( std::vector<candidateCameraView> views)
{
    candidateCameraView bestView ;
    bestView = octomap_local.GetBestCameraView(views);
    geometry_msgs::Point viewPoint;
    viewPoint.x = bestView.x;
    viewPoint.y = bestView.y;
    viewPoint.z = bestView.z;
    point_cloud_marker.points.push_back(viewPoint);
    marker_publisher.publish(point_cloud_marker);
    std::cout << viewPoint.x << " " << viewPoint.y << " " << viewPoint.z << std::endl;
    geometry_msgs::Pose bestPose;
    target_pose.position.x = bestView.x;
    target_pose.position.y = bestView.y;
    target_pose.position.z = bestView.z;
    robot_state_scan = move_to_pose;
    //MoveToPose();
}

void generateCandiadateViews()
{
    std::cout<<"Candidate generation START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    float r = 0.5;
    float xOffset = 1.5, yOffset = 0.0 , zOffset = 0.5;
    float verticalStep = 30.0, horizontalStep = 30.0;
    std::vector<candidateCameraView> views;

    for(float s = 0; s < 360; s+= verticalStep)
    {
        for(float t = 30; t < 360; t += horizontalStep)
        {
            candidateCameraView CAV(r, s * PI/180, t * PI/180,xOffset, yOffset, zOffset);
            views.push_back(CAV);
            position_marker.points.push_back(CAV.GetMessagePoint());
        }
    }
    position_publisher.publish(position_marker);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Candidate generation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    robot_state_scan = evaluating_view;
    evaluateCameraViews(views);
}

void makeScan()
{
    std::cout<<"Making Scan START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(int i = 0; i < current_scan.maped_points.size(); i++)
    {
        map_points_list.maped_points.push_back(current_scan.maped_points.at(i));
    }
    UpdateMarker();
    robot_state_scan = generate_candidate_view;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Making Scan END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    //octomap.WriteScanResults(moveControlClass.GetCameraPoint(),MsgVecToVec3(current_scan.maped_points));
    generateCandiadateViews();
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    //write scan to map
    if(robot_state_scan == initial_scan || robot_state_scan == making_scan) {
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
    if(robot_state_scan == initial_scan || robot_state_scan == making_scan)
    {
        //TODO:uncomment after debug
        makeScan();
    }

    // Publish the data.
    //pub.publish (output);
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    actionlib_msgs::GoalStatusArray goal = *msg;
    if(goal.status_list.at(goal.status_list.size() - 1).status == actionlib_msgs::GoalStatus::SUCCEEDED)
    {
        std::cout << "SUCCEEDED MOVE" << std::endl;
    }
}

int main( int argc, char** argv )
{
    robot_state_scan = initial_scan;
    ros::init(argc, argv, "start_volumetric_bnv");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    //ros::AsyncSpinner spinner(1);
    // /kinect/depth/points
    ros::Subscriber depth_camera_subscriber = n.subscribe("kinect/depth/points",1, cloud_cb);
    //ros::Subscriber moveFeedback = n.subscribe("move_group/feedback",1, statusCallback);// n.subscribe("move_group/feedback",1, moveFeedback);

    ros::Rate r(1);

    marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    initCloudMarker(point_cloud_marker);

    position_publisher = n.advertise<visualization_msgs::Marker>("position_marker", 0);
    initLineMarker(position_marker);

    /* moveit::planning_interface::MoveGroupInterface *group;
    group = new moveit::planning_interface::MoveGroupInterface("arm_group");
    group->setEndEffectorLink(group->getEndEffectorLink());
    geometry_msgs::PoseStamped current_pose =
            group->getCurrentPose();

    /arker_publisher.publish(point_cloud_marker);
    static const std::string PLANNING_GROUP = "arm_group";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    std::cout << "2 "
              << current_pose.pose.position.z move_group_interface.getCurrentPose("link_6").pose.position.x << std::endl;
    */
    while(ros::ok) {

        if (robot_state_scan == move_to_pose) {
            robot_state_scan = wait_move;
            MoveToPose();
        }

   }
    ros::waitForShutdown();
   //ros::spin();

}


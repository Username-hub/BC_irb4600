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
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
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
ros::Publisher target_pose_publisher;
std::vector<candidateCameraView> views;
geometry_msgs::Pose target_pose;
Vec3f cameraPos;
Vec3f center(1.5,0,0.5);


Octomap octomap_local(Vec3f(2,2,2),Vec3f(-2,-2,0),0.1f);
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

    /*std::vector<Voxel> voxelMap = octomap_local.getVoxelMap();
    geometry_msgs::Point p;
    for(Voxel voxel : voxelMap)
    {
        if(voxel.voxelSrate != unmarked) {
            p.x = voxel.bounds[0].x;
            p.y = voxel.bounds[0].y;
            p.z = voxel.bounds[0].z;
            point_cloud_marker.points.push_back(p);
        }
    }*/
    /*p.x = 2;
    p.y = 2;
    p.z = 2;
    point_cloud_marker.points.push_back(p);
    p.x = -2;
    p.y = -2;
    p.z = 0;
    point_cloud_marker.points.push_back(p);*/
    marker_publisher.publish(point_cloud_marker);

}

void evaluateCameraViews()
{
    candidateCameraView bestView ;
    int bestViewNum;
    bestView = octomap_local.GetBestCameraView(views,bestViewNum);
    views.erase(views.begin() + bestViewNum);
    geometry_msgs::Point viewPoint;
    viewPoint.x = bestView.x;
    viewPoint.y = bestView.y;
    viewPoint.z = bestView.z;
    point_cloud_marker.points.push_back(viewPoint);
    marker_publisher.publish(point_cloud_marker);
    setRotation(target_pose,center);
    std::cout << viewPoint.x << " " << viewPoint.y << " " << viewPoint.z << std::endl;
    //geometry_msgs::Pose bestPose;
    target_pose.position.x = bestView.x;
    target_pose.position.y = bestView.y;
    target_pose.position.z = bestView.z;
    //MoveToPose();
    position_publisher.publish(initPoseMarker(views,center));
    geometry_msgs::PoseStamped target_pose_stamped;
    target_pose_stamped.pose.position.x = target_pose.position.x;
    target_pose_stamped.pose.position.y = target_pose.position.y;
    target_pose_stamped.pose.position.z = target_pose.position.z;
    setRotationStamped(target_pose_stamped,center);
    /*target_pose_stamped.pose.orientation.x = target_pose.orientation.x;
    target_pose_stamped.pose.orientation.y = target_pose.orientation.y;
    target_pose_stamped.pose.orientation.z = target_pose.orientation.z;
    target_pose_stamped.pose.orientation.w = target_pose.orientation.w;*/
    target_pose_stamped.header.frame_id = SOURCE_FRAME;
    target_pose_publisher.publish(target_pose_stamped);
    robot_state_scan = move_to_pose;
}

void MoveToPose()
{
    std::cout << "start move" << std::endl;
    MoveControlClass moveControlClass;
    //moveControlClass.MoveToPoint(target_pose);
    //robot_state_scan = making_scan;
    robot_state_scan = wait_move;
    if(!moveControlClass.MoveToPoint(target_pose,center))
    {
        robot_state_scan = evaluating_view;
        evaluateCameraViews();
    }
}
//TODO: make arrow
//TODO: remove pose marker

void generateCandiadateViews()
{
    std::cout<<"Candidate generation START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    float r = 0.5;
    float xOffset = 1.5, yOffset = 0.0 , zOffset = 0.5;
    float verticalStep = 30.0, horizontalStep = 30.0;


    for(float s = 0; s < 350; s+= verticalStep)
    {
        for(float t = 30; t < 350; t += horizontalStep)
        {
            candidateCameraView CAV(r, s * PI/180, t * PI/180,xOffset, yOffset, zOffset);
            views.push_back(CAV);
            //position_marker.points.push_back(CAV.GetMessagePoint());
        }
    }
    //position_publisher.publish(position_marker);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Candidate generation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    //robot_state_scan = evaluating_view;
}

void makeScan()
{
    std::cout<<"Making Scan START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(int i = 0; i < current_scan.maped_points.size(); i++)
    {
        map_points_list.maped_points.push_back(current_scan.maped_points.at(i));
    }
    //UpdateMarker();
    robot_state_scan = generate_candidate_view;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Making Scan END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    MoveControlClass moveControlClass;
    //octomap_local.WriteScanResults(current_scan.maped_points, moveControlClass.GetCameraPoint());
    octomap_local.WriteScanResults(cameraPos,MsgVecToVec3(current_scan.maped_points));
    UpdateMarker();
    evaluateCameraViews();
    //generateCandiadateViews();
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
        tf_listner.waitForTransform(SOURCE_FRAME, TARGET_FRAME, ros::Time(0), ros::Duration(5.0));
        tf::StampedTransform transform;
        tf_listner.lookupTransform(SOURCE_FRAME, TARGET_FRAME, ros::Time(0), transform);
        cameraPos = Vec3f(transform.getOrigin().x(),transform.getOrigin().y(),
                                      transform.getOrigin().z());
        //MoveControlClass moveControlClass;
        //octomap_local.WriteScanResults(output,cameraPoin3d);
        std::cout << "marker publish" << std::endl;
        marker_publisher.publish(point_cloud_marker);
    }
    //Call initial scan function
    if(robot_state_scan == initial_scan || robot_state_scan == making_scan)
    {
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

    position_publisher = n.advertise<geometry_msgs::PoseArray>("pose_marker", 0);
    //initLineMarker(position_marker);
    target_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("target_pose",0);
    generateCandiadateViews();
    while(ros::ok) {

        if (robot_state_scan == move_to_pose) {
            robot_state_scan = wait_move;
            MoveToPose();
            robot_state_scan = making_scan;
        }
        sleep(1);
   }
    ros::waitForShutdown();
   //ros::spin();

}


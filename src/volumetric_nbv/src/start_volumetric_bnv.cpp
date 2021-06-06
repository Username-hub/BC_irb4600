#include <map>
#include <queue>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include "point_translation.h"
#include "markers_init.h"
#include "ray_box_collider.h"
#include "octomap.h"
#include "octomap.cpp"
#include "candidateCameraView.h"
#include "moveControl.h"

#include <pcl/point_cloud.h>

#ifndef SOURCE_FRAME
#define SOURCE_FRAME "base_link"
#endif
#ifndef TARGET_FRAME
#define TARGET_FRAME "link_depthtf"
#endif
#define PI 3.14159265

ros::Publisher pub;
visualization_msgs::Marker point_cloud_marker;
visualization_msgs::Marker position_marker;
ros::Publisher marker_publisher;
ros::Publisher position_publisher;
ros::Publisher position_publisher_un;
ros::Publisher target_pose_publisher;
std::vector<candidateCameraView> views;
geometry_msgs::Pose target_pose;
Vec3f cameraPos;
Vec3f center(1.6,0,0.5);
MoveControlClass moveControlClass;


Octomap octomap_local(Vec3f(3,1,0.9),Vec3f(0.2,-1,0),0.1f);
struct map {
    std::vector<geometry_msgs::Point> maped_points;
}map_points_list,current_scan;

enum StateOfRobot{ initial_scan, generate_candidate_view, evaluating_view, move_to_pose, wait_move, making_scan, wait,program_end} robot_state_scan;

void UpdateMarker()
{
    octomap_local.GetMarkedPoints(point_cloud_marker);
    marker_publisher.publish(point_cloud_marker);
}

char evalType = 's';
void evaluateCameraViews()
{

    if(views.size() < 1)
    {
        robot_state_scan = program_end;
        return;
    }
    candidateCameraView bestView ;
    int bestViewNum;
    if(evalType == 'r')
    {
        bestView = octomap_local.GetBestCameraViewRay(views,cameraPos,center,bestViewNum);
    }
    else
    {
        bestView = octomap_local.GetBestCameraViewSphere(views,cameraPos,bestViewNum);
    }

    views.erase(views.begin() + bestViewNum);

    geometry_msgs::Point viewPoint;
    viewPoint.x = bestView.x;
    viewPoint.y = bestView.y;
    viewPoint.z = bestView.z;

    setRotation(target_pose,center);
    std::cout << viewPoint.x << " " << viewPoint.y << " " << viewPoint.z << std::endl;

    target_pose.position.x = bestView.x;
    target_pose.position.y = bestView.y;
    target_pose.position.z = bestView.z;

    position_publisher.publish(initPoseMarker(views,center));
    geometry_msgs::PoseStamped target_pose_stamped;
    target_pose_stamped.pose.position.x = target_pose.position.x;
    target_pose_stamped.pose.position.y = target_pose.position.y;
    target_pose_stamped.pose.position.z = target_pose.position.z;
    setRotationStamped(target_pose_stamped,center);
    target_pose_stamped.header.frame_id = SOURCE_FRAME;
    target_pose_publisher.publish(target_pose_stamped);
    robot_state_scan = move_to_pose;
}
float dist = 0;
void MoveToPose()
{
    std::cout << "start move" << std::endl;
    robot_state_scan = wait_move;
    if(moveControlClass.MoveToPoint(target_pose,center))
    {
        dist += getDistVec3f(cameraPos, Vec3f(target_pose.position.x,target_pose.position.y,target_pose.position.z));
    }

}

float Step = 30.0;
float rad = 1.0;
void generateCandiadateViews()
{
    std::cout<<"Candidate generation START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();


    float xOffset = 1.5, yOffset = 0.0 , zOffset = 0.5;
    std::vector<candidateCameraView> generatedViews;

    for(float s = 30; s < 350; s+= Step)
    {
        for(float t = 30; t < 350; t += Step)
        {
            if(t > 100 && t < 250)
            {
                t = 260;
            }
            candidateCameraView CAV(rad, s * PI/180, t * PI/180,xOffset, yOffset, zOffset);
            generatedViews.push_back(CAV);
        }
    }

    std::vector<candidateCameraView> unreachableViews;
    moveControlClass.getReachablePositions(generatedViews, views,unreachableViews);
    position_publisher_un.publish(initPoseMarker(unreachableViews,center));
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Candidate generation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    robot_state_scan = initial_scan;
}

void makeScan()
{
    std::cout<<"Making Scan START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for(int i = 0; i < current_scan.maped_points.size(); i++)
    {
        map_points_list.maped_points.push_back(current_scan.maped_points.at(i));
    }

    MoveControlClass moveControlClass;
    float scanedPer;
    octomap_local.WriteScanResults(cameraPos,MsgVecToVec3(current_scan.maped_points),scanedPer);
    if(scanedPer > 0.8)
    {
        robot_state_scan = program_end;
        return;
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Making Scan END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    evaluateCameraViews();

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
            }
        }
        tf_listner.waitForTransform(SOURCE_FRAME, TARGET_FRAME, ros::Time(0), ros::Duration(5.0));
        tf::StampedTransform transform;
        tf_listner.lookupTransform(SOURCE_FRAME, TARGET_FRAME, ros::Time(0), transform);
        cameraPos = Vec3f(transform.getOrigin().x(),transform.getOrigin().y(),
                                      transform.getOrigin().z());
        std::cout << "marker publish" << std::endl;
    }
    //Call initial scan function
    if(robot_state_scan == initial_scan || robot_state_scan == making_scan)
    {
        makeScan();
    }
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    actionlib_msgs::GoalStatusArray goal = *msg;
    if(goal.status_list.at(goal.status_list.size() - 1).status == actionlib_msgs::GoalStatus::SUCCEEDED)
    {
        std::cout << "SUCCEEDED MOVE" << std::endl;
    }
}

int main( int argc, char* argv[] )
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    robot_state_scan = initial_scan;
    ros::init(argc, argv, "start_volumetric_bnv");
    std::string param;
    ros::NodeHandle n;
    n.param<float>("radius",rad, 1.0);
    ROS_INFO("Parameter radius: %f", rad);

    n.param<float>("step",Step, 30.0);
    ROS_INFO("Parameter step: %f", Step);

    std::string str;
    n.getParam("eval_type",str);
    evalType = str[0];
    ROS_INFO("Parameter eval_type: %c", evalType);

    float distWeigth,voxelWeigth;
    n.param<float>("weigth_dist",distWeigth, 0.0);
    ROS_INFO("Parameter weigth_dist: %f", distWeigth);

    n.param<float>("weigth_voxel",voxelWeigth, 1.0);
    ROS_INFO("Parameter weigth_voxel: %f", voxelWeigth);
    octomap_local.setWeights(distWeigth,voxelWeigth);

    int numberOfPos = 0;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber depth_camera_subscriber = n.subscribe("kinect/depth/points",1, cloud_cb);

    ros::Rate r(1);

    marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    initCloudMarker(point_cloud_marker);

    position_publisher = n.advertise<geometry_msgs::PoseArray>("pose_marker", 0);
    position_publisher_un = n.advertise<geometry_msgs::PoseArray>("pose_marker_un", 0);
    target_pose_publisher = n.advertise<geometry_msgs::PoseStamped>("target_pose",0);
    robot_state_scan = generate_candidate_view;
    while(ros::ok) {
        if (robot_state_scan == generate_candidate_view) {
            robot_state_scan = wait;
            generateCandiadateViews();
        }
        if (robot_state_scan == move_to_pose) {
            robot_state_scan = wait_move;
            MoveToPose();
            numberOfPos++;
            robot_state_scan = making_scan;
        }
        if (robot_state_scan == program_end)
        {
            break;
        }
        sleep(2);
   }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Program time" << " after: " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << "[s]" <<
    "Number of pos: " << numberOfPos <<std::endl
    << " Distance: " << dist << std::endl;

}


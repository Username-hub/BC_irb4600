//
// Created by user on 24/03/2021.
//

#ifndef SRC_MOVECONTROL_H
#define SRC_MOVECONTROL_H

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "ray_box_collider.h"


class MoveControlClass {
private:

public:
    MoveControlClass()
    {
    }
    void MoveToPoint(geometry_msgs::Pose aimPose)
    {


        static const std::string PLANNING_GROUP = "robot";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

        const moveit::core::JointModelGroup* joint_model_group =
                move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        move_group.clearPoseTarget();

        geometry_msgs::PoseStamped_<std::allocator<void>> poseStamped = move_group.getCurrentPose(move_group.getEndEffectorLink());

        /*
         * aimPose.orientation.x = poseStamped.pose.orientation.x;
        aimPose.orientation.w = poseStamped.pose.orientation.w;
        aimPose.orientation.y = poseStamped.pose.orientation.y;
        aimPose.orientation.z = poseStamped.pose.orientation.z;
         */
        std::cout << "Aim position: " << aimPose.position.x << " / "
                << aimPose.position.y << " / "
                << aimPose.position.z << " / "
                << std::endl << " Aim orientation: "
                << aimPose.orientation.x << " / "
                << aimPose.orientation.w << " / "
                << aimPose.orientation.y << " / "
                << aimPose.orientation.z << " / "
                << std::endl;
        move_group.setPoseTarget(aimPose);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        //move_group.plan(my_plan);

        moveit::planning_interface::MoveItErrorCode result = move_group.move();
        std::cout << "Result " << result.val << std::endl;

    }
    Vec3f GetCameraPoint()
    {
        static const std::string PLANNING_GROUP = "arm_group";
        moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        geometry_msgs::PoseStamped_<std::allocator<void>> poseStamped = move_group_interface.getCurrentPose(move_group_interface.getEndEffectorLink());
        Vec3f result;
        result.x = poseStamped.pose.position.x;
        result.y = poseStamped.pose.position.y;
        result.z = poseStamped.pose.position.z;
        return result;
    }

};
#endif //SRC_MOVECONTROL_H

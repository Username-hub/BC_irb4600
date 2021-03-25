//
// Created by user on 24/03/2021.
//

#ifndef SRC_MOVECONTROL_H
#define SRC_MOVECONTROL_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "ray_box_collider.h"


class MoveControlClass {
private:

public:
    MoveControlClass()
    {
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

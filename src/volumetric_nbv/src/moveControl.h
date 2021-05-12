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
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include "ray_box_collider.h"

#ifndef SOURCE_FRAME
#define SOURCE_FRAME "base_link"
#endif

class MoveControlClass {
private:

public:
    MoveControlClass()
    {

        //move_group =  moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
        //move_group (PLANNING_GROUP);
    }
    bool MoveToPoint(geometry_msgs::Pose aimPose,const Vec3f &center)
    {

        static const std::string PLANNING_GROUP = "robot";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.25;
        primitive.dimensions[primitive.BOX_Y] = 0.25;
        primitive.dimensions[primitive.BOX_Z] = 0.5;

        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 1.5;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.5;

        move_group.clearPoseTarget();

        //geometry_msgs::PoseStamped_<std::allocator<void>> poseStamped = move_group.getCurrentPose(move_group.getEndEffectorLink());

        setRotation(aimPose,center);
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

        /*moveit_msgs::Constraints constr;
        moveit_msgs::PositionConstraint  positionConstraint;
        positionConstraint.constraint_region.primitives.push_back(primitive);
        constr.position_constraints.push_back(positionConstraint);
        move_group.setPathConstraints(constr);*/
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        //move_group.plan(my_plan);

        moveit::planning_interface::MoveItErrorCode result = move_group.move();
        std::cout << "Result " << result.val << std::endl;
        return true;

    }
    void getReachablePositions(const std::vector<candidateCameraView> &generatedViews,
                               std::vector<candidateCameraView> &Reachable,
                               std::vector<candidateCameraView> &Unreachable)
    {
        static const std::string PLANNING_GROUP = "robot";
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        const moveit::core::JointModelGroup* joint_model_group =
                move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        moveit::core::RobotModelConstPtr kinematic_model(move_group.getRobotModel());
        moveit::core::RobotState rs(kinematic_model);

        kinematics::KinematicsQueryOptions kinematicsQueryOptions;

        //moveit::core::GroupStateValidityCallbackFn groupStateValidityCallbackFn;
        planning_scene::PlanningScene planning_scene(kinematic_model);
        moveit::core::GroupStateValidityCallbackFn groupStateValidityCallbackFn = boost::bind(&MoveControlClass::isIKSolutionValid,this, &planning_scene,_1,_2,_3);


        for(candidateCameraView view : generatedViews)
        {
            if(rs.setFromIK(joint_model_group,view.GetMsgPose(),1.0,groupStateValidityCallbackFn,kinematicsQueryOptions))
            {
                Reachable.push_back(view);
            }
            else
            {
                Unreachable.push_back(view);
            }
        }
    }

    const bool isIKSolutionValid(const planning_scene::PlanningScene* planning_scene,
                                               robot_state::RobotState* state,
                                               const robot_model::JointModelGroup* jmg,
                                               const double* ik_solution)
    {
        state->setJointGroupPositions(jmg, ik_solution);
        state->update();
        return (!planning_scene || !planning_scene->isStateColliding(*state, jmg->getName()));

    }

    /*bool groupStateValidityTest()
    {
        return true;
    }*/

};
#endif //SRC_MOVECONTROL_H

//
// Created by user on 21/03/2021.
//

#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include "ray_box_collider.h"
#ifndef SRC_POINT_TRANSLATION_H
#define SRC_POINT_TRANSLATION_H

void pixelTo3DPoint(const sensor_msgs::PointCloud2 &pCloud, const int u, const int v, geometry_msgs::Point &p)
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

std::vector<Vec3f> MsgVecToVec3(std::vector<geometry_msgs::Point> &maped_points)
{
    std::cout<<"Transform MsgVecToVec3 START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::vector<Vec3f> result;
    for(int i = 0; i < maped_points.size(); i++)
    {
        Vec3f vec3F(maped_points[i].x,maped_points[i].y,maped_points[i].z);
        result.push_back(vec3F);
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Transform MsgVecToVec3 END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    return result;
}


void setRotation(geometry_msgs::Pose &aimPose, const Vec3f &scanAim)
{
    tf::Quaternion q;
    float yaw = atan2(aimPose.position.x - scanAim.x ,aimPose.position.z - scanAim.z);
    float roll = sqrt(pow(aimPose.position.x - scanAim.x,2)+pow(aimPose.position.z - scanAim.z,2));
    float pitch = atan2(roll,aimPose.position.y - scanAim.y);
    q.setEuler(yaw,pitch,-1.571);
    q.normalize();
    aimPose.orientation.x = q.getX();
    aimPose.orientation.y = q.getY();
    aimPose.orientation.z = q.getZ();
    aimPose.orientation.w = q.getW();
}

void setRotationStamped(geometry_msgs::PoseStamped &aimPose, const Vec3f &scanAim)
{
    tf::Quaternion q;
    float yaw = atan2(aimPose.pose.position.x - scanAim.x ,aimPose.pose.position.z - scanAim.z);
    float roll = sqrt(pow(aimPose.pose.position.x - scanAim.x,2)+pow(aimPose.pose.position.z - scanAim.z,2));
    float pitch = atan2(roll,aimPose.pose.position.y - scanAim.y);
    q.setEuler(yaw,pitch,-1.571);
    q.normalize();
    aimPose.pose.orientation.x = q.getX();
    aimPose.pose.orientation.y = q.getY();
    aimPose.pose.orientation.z = q.getZ();
    aimPose.pose.orientation.w = q.getW();
}

#endif //SRC_POINT_TRANSLATION_H
//
// Created by user on 21/03/2021.
//

#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <cmath>
#include "ray_box_collider.h"
#ifndef SRC_POINT_TRANSLATION_H
#define SRC_POINT_TRANSLATION_H
#endif //SRC_POINT_TRANSLATION_H

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
struct Quaternion
{
    double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

/*int dotProduct(int vect_A[], int vect_B[])
{

    int product = 0;
    int n = 3;
    // Loop for calculate cot product
    for (int i = 0; i < n; i++)

        product = product + vect_A[i] * vect_B[i];
    return product;
}*

// Function to find
// cross product of two vector array.
void crossProduct(int vect_A[], int vect_B[], int cross_P[])

{

    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}*/

void setRotation(geometry_msgs::Pose &aimPose, Vec3f scanAim)
{
    /*Vec3f ang;
    ang.x = 0; //atan2(scanAim.z - aimPose.position.z, scanAim.y - aimPose.position.y );
    ang.y = atan2(scanAim.x - aimPose.position.x, scanAim.z - aimPose.position.z );
    ang.z = atan(sqrt(pow(scanAim.y - aimPose.position.y,2) + pow(scanAim.x - aimPose.position.x, 2))/(scanAim.z - aimPose.position.z) );
    Quaternion q = ToQuaternion(ang.x, ang.y, ang.z);

    */

    float d = scanAim.x * aimPose.position.x
             + scanAim.y * aimPose.position.y
             + scanAim.z * aimPose.position.z;
    Vec3f axis;
    axis.x = aimPose.position.y * scanAim.z - aimPose.position.z * scanAim.y;
    axis.y = aimPose.position.z * scanAim.x - aimPose.position.x * scanAim.z;
    axis.z = aimPose.position.x * scanAim.y - aimPose.position.y * scanAim.x;

    aimPose.orientation.z = axis.z;
    aimPose.orientation.x = axis.x;
    aimPose.orientation.y = axis.y;
    Vec3f pose(aimPose.position.x, aimPose.position.y, aimPose.position.z);
    Vec3f aim(scanAim.x, scanAim.y, scanAim.z);
    aimPose.orientation.w = sqrt((pose.norm()*pose.norm()*aim.norm()*aim.norm())+d);
}
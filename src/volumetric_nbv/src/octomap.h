//
// Created by user on 21/03/2021.
//
#include "ray_box_collider.h"
#include "candidateCameraView.h"
#include <list>
#include <iterator>
#include <vector>
#include <thread>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "point_translation.h"

#ifndef SRC_OCTOMAP_H
#define SRC_OCTOMAP_H



enum VoxelState{
    unmarked,
    occupied,
    none,
    occlude,
    occplane
};
class Voxel : public Box3 {

public:
    VoxelState voxelSrate;

    Voxel(const Vec3f &min, const Vec3f &max) : Box3(min, max) {
        bounds[0] = min;
        bounds[1] = max;
        voxelSrate = unmarked;
    }
};

class Octomap
{
private:
    std::vector<Voxel> voxelMap;
    float radius = 1.1;
    float maxVoxelPerView;
    float maxDist = 1.5;
    float distanceWeight = 0.2;
    float voxelWeight = 1.0;
    int maxVoxelPerViewRay;
public:

    Octomap(Vec3f max, Vec3f min, float side);

    void SetRadius(float r);

    void setWeights(float d, float n);

    float SetMaxDist(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos);

    void GetMarkedPoints(visualization_msgs::Marker &point_cloud_marker);

    static void writeResThread(std::vector<Voxel> &voxelMap,const Vec3f &camera_pos, const std::vector<Vec3f> &scan_points, int startNum,int endNum);

    void WriteScanResults(const Vec3f &camera_pos, const std::vector<Vec3f> &scan_points, float &scanedPer);

    candidateCameraView GetBestCameraViewSphere(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos, int &pos);

    candidateCameraView GetBestCameraViewRay(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos,const Vec3f &center, int &pos);
};

#endif //SRC_OCTOMAP_H


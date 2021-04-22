//
// Created by user on 21/03/2021.
//
#include "ray_box_collider.h"
#include "candidateCameraView.h"
#include <list>
#include <iterator>
#include <vector>
#include <octomap/octomap.h>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#ifndef SRC_OCTOMAP_H
#define SRC_OCTOMAP_H

#endif //SRC_OCTOMAP_H

enum VoxelState{
    unmarked,
    occupied,
    none,
    occlude,
    occplane
};

/*class Voxel : public Box3 {

public:
    VoxelState voxelSrate;

    Voxel(const Vec3f &min, const Vec3f &max) : Box3(min, max) {
        bounds[0] = min;
        bounds[1] = max;
        voxelSrate = unmarked;
    }
};*/

class Octomap
{
private:
    octomap::OcTree OCMap;
    octomap::KeyBoolMap KBM;
public:
    Octomap() : OCMap(0.1)
    {
        OCMap.create();
    }
    void WriteScanResults(const sensor_msgs::PointCloud2 &cloud2, const octomap::point3d &cameraPoint)
    {
        std::cout<<"Write scan results START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        octomap::Pointcloud octomapPointcloud;
        pointCloud2ToOctomap(cloud2, octomapPointcloud);
        OCMap.insertPointCloud(octomapPointcloud,cameraPoint, -1, false, false);
        //OCMap.insertPointCloud(&SMPointCloud,)
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Write scan results END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    }

    void pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud, octomap::Pointcloud& octomapCloud) {
        octomapCloud.reserve(cloud.data.size() / cloud.point_step);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            // Check if the point is invalid
            if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z))
                octomapCloud.push_back(*iter_x, *iter_y, *iter_z);
        }
    }
    candidateCameraView GetBestCameraView(const std::vector<candidateCameraView> &views, const Vec3f &ceterVec3f)
    {
        std::cout<<"View evaluation START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        candidateCameraView result = views[0];
        float resultMark = 0;
        octomap::point3d centerOct(ceterVec3f.x,ceterVec3f.y,ceterVec3f.z);
        for(int i = 0; i < views.size(); i++)
        {
            octomap::point3d viewPointOct(views[i].x,views[i].y,views[i].z);
            octomap::KeyRay keyRay;
            OCMap.computeRayKeys(viewPointOct,centerOct,keyRay);
            //std::cout << "Key ray: " << keyRay.size() << std::endl;
            octomap::KeyRay::iterator ray_iter;
            float mark = 0;
            for(ray_iter = keyRay.begin();ray_iter < keyRay.end(); ray_iter++)
            {
                octomap::OcTreeKey ocTreeKey;
                ocTreeKey = *ray_iter.base();
                //std::cout << "OcTreeKey: " << ray_iter->k << " " << ocTreeKey.k[1] << " " << ocTreeKey.k[1] << " " << std::endl;
                octomap::OcTreeNode *ocTreeNode = OCMap.search(ocTreeKey, 0);
                //std::cout << "ocTreeNode: " << ocTreeNode->getLogOdds() << std::endl;
                if(ocTreeNode) {
                    mark += ocTreeNode->getLogOdds();
                }
            }
            if(resultMark < mark)
            {
                result = views[i];
                resultMark = mark;
            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"View evaluation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        return result;
    }
};


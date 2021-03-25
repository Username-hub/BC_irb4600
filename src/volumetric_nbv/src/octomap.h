//
// Created by user on 21/03/2021.
//
#include "ray_box_collider.h"
#include "candidateCameraView.h"
#include <list>
#include <iterator>
#include <vector>

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

public:
    Octomap(Vec3f max, Vec3f min, float side)
    {
        std::cout<<"Octomap generation START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        for(float x = min.x; x <= max.x; x+=side)
        {
            for(float y = min.y; y <= max.y; y+=side)
            {
                for(float z = min.z; z <= max.z; z+=side)
                {
                    voxelMap.push_back(Voxel(Vec3f(x,y,z),Vec3f(x+side,y+side,z+side)));
                }
            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Octomap generation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    }

    bool GetVoxelByPoint(Voxel &result,Vec3f point)
    {
        for (int i = 0; i < voxelMap.size(); i++) {
            if((point.x > voxelMap[i].bounds[0].x && point.x < voxelMap[i].bounds[1].x ) &&
                (point.y > voxelMap[i].bounds[0].y && point.y < voxelMap[i].bounds[1].y ) &&
                (point.z > voxelMap[i].bounds[0].z && point.z < voxelMap[i].bounds[1].z ))
            {
                result = voxelMap[i];
                return true;
            }
        }
        return false;
    }

    void WriteScanResults(Vec3f camera_pos, std::vector<Vec3f> scan_points)
    {
        std::cout<<"Write scan results START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        for(int vectorCounter = 0; vectorCounter < scan_points.size(); vectorCounter++)
        {
            for(int boxCounter = 0; boxCounter < voxelMap.size(); boxCounter++)
            {
                if(voxelMap[boxCounter].voxelSrate != unmarked) continue;
                if(voxelMap[boxCounter].intersect(Ray(camera_pos,scan_points[vectorCounter])))
                {
                    voxelMap[boxCounter].voxelSrate = none;
                }
                if(voxelMap[boxCounter].CheckPoint(scan_points[vectorCounter]))
                {
                    voxelMap[boxCounter].voxelSrate = occupied;
                }
            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Write scan results END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    }

    candidateCameraView GetBestCameraView(std::vector<candidateCameraView> views)
    {
        std::cout<<"Camera view evaluation START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        candidateCameraView result = views[0];
        result.value = 0;
        for(int i =0 ;i < views.size(); i++)
        {
            int mark = 0;
            for(int j =0; j < voxelMap.size() ; j++)
            {
                if(voxelMap[j].voxelSrate == unmarked)
                {
                    if(voxelMap[j].intersect(Ray(views[i].point,views[i].center)))
                    {
                        mark++;
                    }
                }
            }
            views[i].value = mark;
            if(result.value < views[i].value)
            {
                result = views[i];
            }
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Camera view evaluation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        return result;
    }
};


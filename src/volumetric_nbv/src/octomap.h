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
    float radius = 0.25;
    float maxVoxelPerView;
    float maxDist = 4.2;
    float distanceWeight = 0.2;
public:
    const std::vector<Voxel> &getVoxelMap() const {
        return voxelMap;
    }

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
        maxVoxelPerView = 0;
        Vec3f center(1.25,0,0.6);
        for(int j =0; j < voxelMap.size() ; j++)
        {
            if(voxelMap[j].intersectsWith(Sphere(center,radius)))
            {
                maxVoxelPerView++;
            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Octomap generation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    }

    void SetRadius(float r)
    {
        radius = r;
        maxVoxelPerView = 0;
        Vec3f center(1.25,0,0.6);
        for(int j =0; j < voxelMap.size() ; j++)
        {
            if(voxelMap[j].intersectsWith(Sphere(center,radius)))
            {
                maxVoxelPerView++;
            }
        }
    }
    void setDistWeight(float d)
    {
        distanceWeight = d;
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

    void GetMarkedPoints(visualization_msgs::Marker &point_cloud_marker)
    {
        point_cloud_marker.points.clear();
        for(Voxel voxel : voxelMap)
        {
            if(voxel.voxelSrate == occupied)
            {
                geometry_msgs::Point p;
                p.x = voxel.bounds[0].x;
                p.y = voxel.bounds[0].y;
                p.z = voxel.bounds[0].z;
                point_cloud_marker.points.push_back(p);
            }
        }
    }

    static void writeResThread(std::vector<Voxel> &voxelMap,const Vec3f &camera_pos, const std::vector<Vec3f> &scan_points, int startNum,int endNum)
    {
        for(int vectorCounter = 0; vectorCounter < scan_points.size(); vectorCounter+=2)
        {
            for(int boxCounter = startNum; boxCounter < endNum; boxCounter++)
            {
                if(voxelMap[boxCounter].CheckPoint(scan_points[vectorCounter]))
                {

                    voxelMap[boxCounter].voxelSrate = occupied;
                    continue;
                }
                if(voxelMap[boxCounter].voxelSrate != unmarked) {

                    continue;
                }

                //if(voxelMap[boxCounter].intersect(Ray(camera_pos,scan_points[vectorCounter])))
                Vec3f Hit;
                if(voxelMap[boxCounter].CheckLineBox(camera_pos,scan_points[vectorCounter],Hit))
                {

                    voxelMap[boxCounter].voxelSrate = none;
                }
            }
        }
    }

    void WriteScanResults(const Vec3f &camera_pos, const std::vector<Vec3f> &scan_points)
    {
        std::cout<<"Write scan results START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        int startNum = 0 , endNum = voxelMap.size()/4 - 1;
        std::thread th1(writeResThread,std::ref(voxelMap),camera_pos,scan_points,startNum,endNum);
        startNum = endNum + 1;
        endNum = startNum + voxelMap.size()/4 - 1;
        std::thread th2(writeResThread,std::ref(voxelMap),camera_pos,scan_points,startNum,endNum);
        startNum = endNum + 1;
        endNum = startNum + voxelMap.size()/4 - 1;
        std::thread th3(writeResThread,std::ref(voxelMap),camera_pos,scan_points,startNum,endNum);
        startNum = endNum + 1;
        endNum = startNum + voxelMap.size()/4 - 1;
        std::thread th4(writeResThread,std::ref(voxelMap),camera_pos,scan_points,startNum,endNum);
        //std::thread th1(foo,1);
        int unm = 0, noneVex = 0, occ = 0;

        th1.join();
        th2.join();
        th3.join();
        th4.join();
        for(int boxCounter = 0; boxCounter < voxelMap.size(); boxCounter++)
        {
            switch (voxelMap[boxCounter].voxelSrate) {
                case unmarked:
                    unm++;
                    break;
                case none:
                    noneVex++;
                    break;
                case occupied:
                    occ++;
                    break;
            }
        }
        std::cout << "Unmarked: " << unm << " / None: " << noneVex << " / Occupied: " << occ << std::endl;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Write scan results END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    }

    candidateCameraView GetBestCameraViewSphere(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos, int &pos)
    {

        std::cout<<"Sphere Camera view evaluation START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        candidateCameraView result = views[0];

        float currentBest = 0;
        for(int i =0 ;i < views.size(); i++)
        {
            float mark = 0;
            for(int j =0; j < voxelMap.size() ; j++)
            {
                if(voxelMap[j].voxelSrate == unmarked)
                {
                    //if(voxelMap[j].intersect(Ray(views[i].point,views[i].center)))
                    if(voxelMap[j].intersectsWith(Sphere(views[i].point,radius)))
                    {
                        mark++;
                    }
                }
            }
            //scale  variable 𝑥 into a range [𝑎,𝑏]
            //𝑥_𝑛𝑜𝑟𝑚𝑎𝑙𝑖𝑧𝑒𝑑=(𝑏−𝑎)(𝑥−𝑚𝑖𝑛(𝑥))/(𝑚𝑎𝑥(𝑥)−𝑚𝑖𝑛(𝑥))+𝑎
            float mark_normalized = (100) * (mark)/(maxVoxelPerView);
            float dist_normalized = (100) * ((getDistVec3f(views[i].point, cameraPos))/maxDist);
            //std::cout << "Mark: " << mark_normalized << " / " << mark << " / " << maxVoxelPerView << " / " << (mark)/(maxVoxelPerView) << std::endl;
            //std::cout << "Mark: " << dist_normalized << " / " << getDistVec3f(views[i].point, cameraPos) << " / " << maxDist << " / " << ((getDistVec3f(views[i].point, cameraPos))/maxDist) << std::endl;
            float finalMark = mark_normalized - distanceWeight * dist_normalized;
            if(currentBest < finalMark)
            {
                result = views[i];
                pos = i;
                currentBest = finalMark;
            }
        }
        std::cout << "Mark: " << currentBest << " / Pos " << pos << std::endl;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Camera view evaluation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        return result;
    }
    candidateCameraView GetBestCameraViewRay(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos,const Vec3f &center, int &pos)
    {
        std::cout<<"RayCamera view evalu ation START"<<std::endl;
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        candidateCameraView result = views[0];

        float currentBest = 0;
        for(int i =0 ;i < views.size(); i++)
        {
            float mark = 0;
            for(int j =0; j < voxelMap.size() ; j++)
            {
                if(voxelMap[j].voxelSrate == unmarked)
                {
                    Vec3f Hit;
                    if(voxelMap[j].CheckLineBox(cameraPos,center,Hit))
                    {
                        mark++;
                    }
                }
            }
            //scale  variable 𝑥 into a range [𝑎,𝑏]
            //𝑥_𝑛𝑜𝑟𝑚𝑎𝑙𝑖𝑧𝑒𝑑=(𝑏−𝑎)(𝑥−𝑚𝑖𝑛(𝑥))/(𝑚𝑎𝑥(𝑥)−𝑚𝑖𝑛(𝑥))+𝑎
            float mark_normalized = (100) * (mark)/(maxVoxelPerView);
            float dist_normalized = (100) * ((getDistVec3f(views[i].point, cameraPos))/maxDist);
            //std::cout << "Mark: " << mark_normalized << " / " << mark << " / " << maxVoxelPerView << " / " << (mark)/(maxVoxelPerView) << std::endl;
            //std::cout << "Mark: " << dist_normalized << " / " << getDistVec3f(views[i].point, cameraPos) << " / " << maxDist << " / " << ((getDistVec3f(views[i].point, cameraPos))/maxDist) << std::endl;
            float finalMark = mark_normalized - distanceWeight * dist_normalized;
            if(currentBest < finalMark)
            {
                result = views[i];
                pos = i;
                currentBest = finalMark;
            }
        }
        std::cout << "Mark: " << currentBest << " / Pos " << pos << std::endl;
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout<<"Camera view evaluation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        return result;
    }
};


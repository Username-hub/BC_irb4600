//
// Created by user on 06/06/2021.
//
#include "octomap.h"

Octomap::Octomap(Vec3f max, Vec3f min, float side) {
    std::cout << "Octomap generation START" << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for (float x = min.x; x <= max.x; x += side) {
        for (float y = min.y; y <= max.y; y += side) {
            for (float z = min.z; z <= max.z; z += side) {
                voxelMap.push_back(Voxel(Vec3f(x, y, z), Vec3f(x + side, y + side, z + side)));
            }
        }
    }
    maxVoxelPerView = 0;
    Vec3f center(1.25, 0, 0.6);
    for (int j = 0; j < voxelMap.size(); j++) {
        if (voxelMap[j].intersectsWith(Sphere(center, radius))) {
            maxVoxelPerView++;
        }
        Vec3f Hit;
        if (voxelMap[j].CheckLineBox(max, min, Hit)) {
            maxVoxelPerViewRay++;
        }
    }
    maxVoxelPerViewRay = maxVoxelPerViewRay / 2;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Octomap generation END" << " after: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}

void Octomap::SetRadius(float r)
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

void Octomap::setWeights(float d, float n)
{
    distanceWeight = d;
    voxelWeight = n;
}

float Octomap::SetMaxDist(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos)
{
    float res = 0;
    for(candidateCameraView v : views)
    {
        float x = getDistVec3f(v.point,cameraPos);
        if(x > res)
            res = x;
    }
    return res;
}

void Octomap::GetMarkedPoints(visualization_msgs::Marker &point_cloud_marker)
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

void Octomap::writeResThread(std::vector<Voxel> &voxelMap,const Vec3f &camera_pos, const std::vector<Vec3f> &scan_points, int startNum,int endNum)
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

            Vec3f Hit;
            if(voxelMap[boxCounter].CheckLineBox(camera_pos,scan_points[vectorCounter],Hit))
            {

                voxelMap[boxCounter].voxelSrate = none;
            }
        }
    }
}

void Octomap::WriteScanResults(const Vec3f &camera_pos, const std::vector<Vec3f> &scan_points, float &scanedPer)
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
    scanedPer = ((float)(occ + noneVex))/((float)voxelMap.size());
    std::cout << "Unmarked: " << unm << " / None: " << noneVex << " / Occupied: " << occ << " / %: " <<  scanedPer << std::endl;
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Write scan results END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
}

candidateCameraView Octomap::GetBestCameraViewSphere(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos, int &pos)
{

    std::cout << "Views num before: " << views.size() - 1 << std::endl;
    std::cout<<"Sphere Camera view evaluation START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    candidateCameraView result = views[0];
    maxDist = SetMaxDist(views,cameraPos);
    float currentBest = 0 - maxDist;
    pos = 0;
    for(int i =0 ;i < views.size(); i++)
    {
        float mark = 0;
        for(int j =0; j < voxelMap.size() ; j++)
        {
            if(voxelMap[j].voxelSrate == unmarked)
            {
                if(voxelMap[j].intersectsWith(Sphere(views[i].point,radius)))
                {
                    mark++;
                }
            }
        }
        //scale  variable 𝑥 into a range [𝑎,𝑏]
        //𝑥_𝑛𝑜𝑟𝑚𝑎𝑙𝑖𝑧𝑒𝑑=(𝑏−𝑎)(𝑥−𝑚𝑖𝑛(𝑥))/(𝑚𝑎𝑥(𝑥)−𝑚𝑖𝑛(𝑥))+𝑎
        float mark_normalized = (mark)/(maxVoxelPerView);
        float dist_normalized =  ((getDistVec3f(views[i].point, cameraPos))/maxDist);

        if(mark_normalized > 1.0 || mark_normalized < 0 || dist_normalized > 1.0 || dist_normalized < 0)
        {
            std::cout << "Mark: " << mark_normalized << " / " << mark << " / " << maxVoxelPerView << " / " << (mark)/(maxVoxelPerView) << std::endl;
            std::cout << "Mark: " << dist_normalized << " / " << getDistVec3f(views[i].point, cameraPos) << " / " << maxDist << " / " << ((getDistVec3f(views[i].point, cameraPos))/maxDist) << std::endl;

        }
        float finalMark = voxelWeight * mark_normalized - distanceWeight * dist_normalized;
        if(currentBest < finalMark)
        {
            result = views[i];
            pos = i;
            currentBest = finalMark;
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Camera view evaluation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    std::cout << "Views num after: " << views.size() - 1 << std::endl;
    return result;
}

candidateCameraView Octomap::GetBestCameraViewRay(const std::vector<candidateCameraView> &views,const Vec3f &cameraPos,const Vec3f &center, int &pos)
{
    std::cout<<"RayCamera view evalu ation START"<<std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    candidateCameraView result = views[0];

    maxDist = SetMaxDist(views,cameraPos);
    float currentBest = 0 - maxDist;
    pos = 0;
    for(int i =0 ;i < views.size(); i++)
    {
        float mark = 0;
        for(int j =0; j < voxelMap.size() ; j++)
        {
            if(voxelMap[j].voxelSrate == unmarked)
            {
                Vec3f Hit;
                if(voxelMap[j].CheckLineBox(views[i].point,center,Hit))
                {
                    mark++;
                }
            }
        }
        //scale  variable 𝑥 into a range [𝑎,𝑏]
        //𝑥_𝑛𝑜𝑟𝑚𝑎𝑙𝑖𝑧𝑒𝑑=(𝑏−𝑎)(𝑥−𝑚𝑖𝑛(𝑥))/(𝑚𝑎𝑥(𝑥)−𝑚𝑖𝑛(𝑥))+𝑎
        float mark_normalized = (mark)/(maxVoxelPerViewRay);
        float dist_normalized =  ((getDistVec3f(views[i].point, cameraPos))/maxDist);
        if(mark_normalized > 1.0 || mark_normalized < 0 || dist_normalized > 1.0 || dist_normalized < 0)
        {
            std::cout << "Mark: " << mark_normalized << " / " << mark << " / " << maxVoxelPerView << " / " << (mark)/(maxVoxelPerView) << std::endl;
            std::cout << "Mark: " << dist_normalized << " / " << getDistVec3f(views[i].point, cameraPos) << " / " << maxDist << " / " << ((getDistVec3f(views[i].point, cameraPos))/maxDist) << std::endl;

        }
        float finalMark = voxelWeight * mark_normalized - distanceWeight * dist_normalized;
        if(currentBest < finalMark)
        {
            result = views[i];
            pos = i;
            currentBest = finalMark;
        }
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout<<"Camera view evaluation END" << " after: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    return result;
}
//
// Created by user on 22/03/2021.
//

#ifndef SRC_CANDIDATECAMERAVIEW_H
#define SRC_CANDIDATECAMERAVIEW_H
class candidateCameraView
{
public:
    float x;
    float y;
    float z;
    Vec3f point;
    Vec3f center;
    int value;
    candidateCameraView()
    {

    }
    candidateCameraView(float r, float s, float t, float xOffset, float yOffset, float zOffset)
    {
        x = r * cos(s) * sin(t) + xOffset;
        y = r * sin(s) * sin(t) + yOffset;
        z = r * cos(t) + zOffset;
        point.x = x;
        point.y = y;
        point.z = z;
        center.x = xOffset;
        center.y = yOffset;
        center.z = zOffset;
    }

    geometry_msgs::Point GetMessagePoint()
    {
        geometry_msgs::Point result;
        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    }
};
#endif //SRC_CANDIDATECAMERAVIEW_H

//
// Created by user on 21/03/2021.
//

#ifndef SRC_RAY_BOX_COLLIDER_H
#define SRC_RAY_BOX_COLLIDER_H

#endif //SRC_RAY_BOX_COLLIDER_H

class Vector3 {
public:
    float x() const {return xPoint;}
    float y() const {return yPoint;}
    float z() const {return zPoint;}

    Vector3(float xIn, float yIn, float zIn)
    {
        xPoint = xIn;
        yPoint = yIn;
        zPoint = zIn;
    }

    Vector3() {}

private:
    float xPoint,yPoint,zPoint;
};
class Ray {
public:
    Ray(Vector3 &o, Vector3 &d)
    {
        origin = o;
        direction = d;
        inv_direction = Vector3(1/d.x(), 1/d.y(), 1/d.z());
        sign[0] = (inv_direction.x() < 0);
        sign[1] = (inv_direction.y() < 0);
        sign[2] = (inv_direction.z() < 0);
    }
    Vector3 origin;
    Vector3 direction;
    Vector3 inv_direction;
    int sign[3];
};

class Box {
public:
    Box(const Vector3 &min, const Vector3 &max)
    {
        //assert(min < max);
        bounds[0] = min;
        bounds[1] = max;
    }
    bool intersect(const Ray &, float t0, float t1) const;
    Vector3 bounds[2];
};
// Smits’ method
bool Box::intersect(const Ray &r, float t0, float t1) const
{
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    if (r.direction.x() >= 0)
    {
        tmin = (bounds[0].x() - r.origin.x()) / r.direction.x();
        tmax = (bounds[1].x() - r.origin.x()) / r.direction.x();
    }else {
        tmin = (bounds[1].x() - r.origin.x()) / r.direction.x();
        tmax = (bounds[0].x() - r.origin.x()) / r.direction.x();
    }
    if (r.direction.y() >= 0)
    {
        tymin = (bounds[0].y() - r.origin.y()) / r.direction.y();
        tymax = (bounds[1].y() - r.origin.y()) / r.direction.y();
    }else {
        tymin = (bounds[1].y() - r.origin.y()) / r.direction.y();
        tymax = (bounds[0].y() - r.origin.y()) / r.direction.y();
    }if ( (tmin > tymax) || (tymin > tmax) )
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    if (r.direction.z() >= 0)
    {
        tzmin = (bounds[0].z() - r.origin.z()) / r.direction.z();
        tzmax = (bounds[1].z() - r.origin.z()) / r.direction.z();
    }else {
        tzmin = (bounds[1].z() - r.origin.z()) / r.direction.z();
        tzmax = (bounds[0].z() - r.origin.z()) / r.direction.z();
    }
    if ( (tmin > tzmax) || (tzmin > tmax) )
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    return ( (tmin < t1) && (tmax > t0) );
}
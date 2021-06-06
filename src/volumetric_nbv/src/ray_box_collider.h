//
// Created by user on 21/03/2021.
//
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>
#include <utility>
#include <cstdint>
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>
#include <random>

#ifndef SRC_RAY_BOX_COLLIDER_H
#define SRC_RAY_BOX_COLLIDER_H

template<typename T>
class Vec3
{
public:
    Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
    Vec3(T xx) : x(xx), y(xx), z(xx) {}
    Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}
    Vec3 operator + (const Vec3 &v) const
    { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator - (const Vec3 &v) const
    { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator - () const
    { return Vec3(-x, -y, -z); }
    Vec3 operator * (const T &r) const
    { return Vec3(x * r, y * r, z * r); }
    Vec3 operator * (const Vec3 &v) const
    { return Vec3(x * v.x, y * v.y, z * v.z); }
    T dotProduct(const Vec3<T> &v) const
    { return x * v.x + y * v.y + z * v.z; }
    Vec3& operator /= (const T &r)
    { x /= r, y /= r, z /= r; return *this; }
    Vec3& operator *= (const T &r)
    { x *= r, y *= r, z *= r; return *this; }
    Vec3 crossProduct(const Vec3<T> &v) const
    { return Vec3<T>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    T norm() const
    { return x * x + y * y + z * z; }
    T length() const
    { return sqrt(norm()); }
    const T& operator [] (uint8_t i) const { return (&x)[i]; }
    T& operator [] (uint8_t i) { return (&x)[i]; }
    Vec3& normalize()
    {
        T n = norm();
        if (n > 0) {
            T factor = 1 / sqrt(n);
            x *= factor, y *= factor, z *= factor;
        }

        return *this;
    }

    friend Vec3 operator * (const T &r, const Vec3 &v)
    { return Vec3<T>(v.x * r, v.y * r, v.z * r); }
    friend Vec3 operator / (const T &r, const Vec3 &v)
    { return Vec3<T>(r / v.x, r / v.y, r / v.z); }

    friend std::ostream& operator << (std::ostream &s, const Vec3<T> &v)
    {
        return s << '[' << v.x << ' ' << v.y << ' ' << v.z << ']';
    }

    T x, y, z;
};
typedef Vec3<float> Vec3f;
typedef Vec3<int> Vec3i;

class Ray
{
public:
    Ray(const Vec3f &orig, const Vec3f &dir) : orig(orig), dir(dir)
    {
        invdir = 1 / dir;
        sign[0] = (invdir.x < 0);
        sign[1] = (invdir.y < 0);
        sign[2] = (invdir.z < 0);
    }
    Vec3f orig, dir;       // ray orig and dir
    Vec3f invdir;
    int sign[3];
};

class Sphere
{
public:
    Vec3f center;
    float radius;

    Sphere(const Vec3f &center, float radius) : center(center), radius(radius) {}
};
class Box3 {
public:
    Box3(const Vec3f &vmin, const Vec3f &vmax) {
        bounds[0] = vmin;
        bounds[1] = vmax;
    }

    Vec3f bounds[2];

    int inline GetIntersection(float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f &Hit) {
        if ((fDst1 * fDst2) >= 0.0f) return 0;
        if (fDst1 == fDst2) return 0;
        Hit = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));
        return 1;
    }


    int inline InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis) {
        if (Axis == 1 && Hit.z > B1.z && Hit.z < B2.z && Hit.y > B1.y && Hit.y < B2.y) return 1;
        if (Axis == 2 && Hit.z > B1.z && Hit.z < B2.z && Hit.x > B1.x && Hit.x < B2.x) return 1;
        if (Axis == 3 && Hit.x > B1.x && Hit.x < B2.x && Hit.y > B1.y && Hit.y < B2.y) return 1;
        return 0;
    }

// returns true if line (L1, L2) intersects with the box (B1, B2)
// returns intersection point in Hit
    int CheckLineBox( Vec3f L1, Vec3f L2, Vec3f &Hit) {
        Vec3f B1 = bounds[0];
        Vec3f B2 = bounds[1];
        if (L2.x < B1.x && L1.x < B1.x) return false;
        if (L2.x > B2.x && L1.x > B2.x) return false;
        if (L2.y < B1.y && L1.y < B1.y) return false;
        if (L2.y > B2.y && L1.y > B2.y) return false;
        if (L2.z < B1.z && L1.z < B1.z) return false;
        if (L2.z > B2.z && L1.z > B2.z) return false;
        if (L1.x > B1.x && L1.x < B2.x &&
            L1.y > B1.y && L1.y < B2.y &&
            L1.z > B1.z && L1.z < B2.z) {
            Hit = L1;
            return true;
        }
        if ((GetIntersection(L1.x - B1.x, L2.x - B1.x, L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y - B1.y, L2.y - B1.y, L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z - B1.z, L2.z - B1.z, L1, L2, Hit) && InBox(Hit, B1, B2, 3))
            || (GetIntersection(L1.x - B2.x, L2.x - B2.x, L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y - B2.y, L2.y - B2.y, L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z - B2.z, L2.z - B2.z, L1, L2, Hit) && InBox(Hit, B1, B2, 3)))
            return true;

        return false;
    }

    bool CheckPoint(Vec3f point)
    {

        if((point.x > bounds[0].x && point.x < bounds[1].x ) &&
           (point.y > bounds[0].y && point.y < bounds[1].y ) &&
           (point.z > bounds[0].z && point.z < bounds[1].z ))
        {
            return true;
        }
        return false;
    }

bool intersectsWith(Sphere sphere) {
        float dmin = 0;

        Vec3f center = sphere.center;
        Vec3f bmin = bounds[0];
        Vec3f bmax = bounds[1];

        if (center.x < bmin.x) {
            dmin += pow(center.x - bmin.x, 2);
        } else if (center.x > bmax.x) {
            dmin += pow(center.x - bmax.x, 2);
        }

        if (center.y < bmin.y) {
            dmin += pow(center.y - bmin.y, 2);
        } else if (center.y > bmax.y) {
            dmin += pow(center.y - bmax.y, 2);
        }

        if (center.z < bmin.z) {
            dmin += pow(center.z - bmin.z, 2);
        } else if (center.z > bmax.z) {
            dmin += pow(center.z - bmax.z, 2);
        }

        return dmin <= pow(sphere.radius, 2);
    }
};

#endif //SRC_RAY_BOX_COLLIDER_H
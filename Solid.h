#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "Light.h"
#include <string>
#include <string.h>

#ifndef Solid_H
#define Solid_H

class Solid;

struct HitRecord
{
    Solid *nearest;
    Eigen::Vector3d color;
    double t;               // how far away from the previous intersection this impact was
    Eigen::Vector3d normal; // normal to the surface where the ray impacted
    Eigen::Vector3d rayDir; // direction of the (incoming) ray
    Eigen::Vector3d e;      // where the ray came from
    Eigen::Vector3d impactPos;
    Eigen::Vector3d reflectionDir; // direction of the (outgoing) ray

    int reflectionsRemaining;
};

struct ObjProps
{
    double r, g, b, Kd, Ks, Shine, T, index_of_refraction;
    Eigen::Vector3d color;

    void setColor()
    {
        color = Eigen::Vector3d{r, g, b};
    }
};

template <typename T>
class Octree;

class Solid
{
public:

    //convenience function
    void printVec(Eigen::Vector3d v)
    {
        printf("[%f, %f, %f]\n", v[0], v[1], v[2]);
    }

    std::vector<Eigen::Vector3d> verts; // only used for polygons and triangles right now
    ObjProps properties;
    double distToCam = -1;
    bool phong = false;
    Solid(ObjProps props) : properties(props)
    {
        properties.setColor();
    }

    Solid(ObjProps props, std::vector<Eigen::Vector3d> verts) : properties(props), verts(verts)
    {
        properties.setColor();
    }

    std::string toString()
    {
        // std::stringstream s;
        // s << verts;
        // return s.str();
        return "Solid has " + std::to_string(verts.size()) + " verticies.";
    }

    virtual double calcDistanceToCam(Eigen::Vector3d &e)
    {
        return -1;
    }

    virtual double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT)
    {
        std::cout << "base class intersect\n";
        return maxT;
    }

    virtual double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, HitRecord &hr)
    {
        std::cout << "base class intersect\n";
        return maxT;
    }

    virtual Eigen::Vector4d raytrace(Eigen::Vector3d rayDir, Eigen::Vector3d e, double minT, double maxT, int reflectionsRemaining, std::vector<Solid *> &solids, Solid *parent)
    {
        std::cout << "base class raytrace\n";
        return Eigen::Vector4d{0, 0, 0, 0};
    }

    virtual bool insideOctree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d center)
    {
        std::cout << "base class inside octree\n";
        return false;
    }

    virtual bool insideOctree(Eigen::Vector3d corner1, Eigen::Vector3d corner2)
    {
        std::cout << "base class inside octree\n";
        return false;
    }

   

    void updateHitRecord(double t, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, Eigen::Vector3d &normal, HitRecord &hr)
    {
        if (t < hr.t)
        {
            hr.color = properties.color;
            hr.t = t;
            hr.nearest = this;
            hr.rayDir = rayDir; // rayDir should already be normalized. (If its not that's a problem!)
            hr.e = e;
            hr.normal = normal.normalized();
            hr.impactPos = e + rayDir * t;
            //taken from textbook
            hr.reflectionDir =  (rayDir + 2.0 * -rayDir.dot(hr.normal) * hr.normal).normalized();

        }
    }

    //  returns a color
    Eigen::Vector3d computeDiffuseComp(Eigen::Vector3d &impactPos, Eigen::Vector3d &normal, Light &l)
    {
        //printf("diff\n");
        Eigen::Vector3d comp{0, 0, 0};

        comp += l.intensity * properties.Kd * properties.color * std::max((l.pos - impactPos).normalized().dot(normal), 0.0);
        //std::cout << comp << std::endl;
        // if(std::max((impactPos - l.pos).dot(normal), 0.0) > .001){
        //     //printf("%f\n", std::max((impactPos - l.pos).dot(normal), 0.0));

        //     //printf("impact pos:\n");
        //     //std::cout << impactPos << std::endl;

        //     //printf("light pos:\n");
        //     //std::cout << l.pos << std::endl;
        // }

        return comp;
    }

    // returns a color
    // e is where the ray intersected the Solid, camPos is where the camera is
    //Note to self: check parameters when calling a func, just wasted 1.5 hours debugging this function b/c of that.
    Eigen::Vector3d computeSpecularComp(HitRecord& hr, Eigen::Vector3d &impactPos, Eigen::Vector3d &e, Eigen::Vector3d &camPos, Eigen::Vector3d &normal, Light &l)
    {
        Eigen::Vector3d n = normal.normalized(); 
        
        Eigen::Vector3d comp{0, 0, 0};
        Eigen::Vector3d v = (e - impactPos).normalized();
        // loop over all the lights, summing their effects

        Eigen::Vector3d lDir = (l.pos - impactPos).normalized();
        
        //halfway angle method
        //Eigen::Vector3d h = (lDir + v).normalized();
        //comp += l.intensity * properties.Ks * l.color * std::pow(std::max(n.dot(h), 0.0), properties.Shine);
        
        //light reflection method
        Eigen::Vector3d reflectionDir = (-lDir + 2.0 * lDir.dot(n) * n).normalized();
        comp += l.intensity * properties.Ks * l.color * std::pow(std::max(reflectionDir.dot(v), 0.0), properties.Shine);
        // printf(" Alignment = %f  ", reflectionDir.dot(v));
        // printVec(comp);

        return comp;
    }

    virtual Eigen::Vector3d getNormal(Eigen::Vector3d & impactPos){
        return Eigen::Vector3d{0,0,0};
    }

    virtual string TYPE(){
        return "Solid";
    }

    virtual int type(){
        return -1;
    }
};

// structs useful for returning intersection info
struct pixelInfo
{
    Solid *nearest;
    double t;
};

struct rtInfo
{
    Solid *nearest;
    Eigen::Vector4d color;
};

#endif


 // checks if point e is in shadow by checking LOS with all the lights in the scene.
    // bool inShadow(Octree *root, Eigen::Vector3d &e, std::vector<Light> &lights, Octree* root)
    // {
    //     bool inShadow = false;
    //     double maxT = 10000000000;
    //     double minT = 0;

    //     // placeholder vars
    //     long long int a;
    //     long int b, c;

    //     // loop over all the lights in the scene
    //     for (Light l : lights)
    //     {
    //         Eigen::Vector3d rayDir = l.pos - e;

    //         // check for intersection of this ray with any solids along the ray starting at the impact point to the light.
    //         if (root->intersect(rayDir, e, minT, maxT, maxT, a, b, c).t != maxT)
    //         {
    //             inShadow = true;
    //             break;
    //         }
    //     }

    //     return inShadow;
    // }
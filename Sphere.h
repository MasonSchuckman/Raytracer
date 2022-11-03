#include "Solid.h"
#include <chrono>

#ifndef Sphere_H
#define Sphere_H

class Sphere : public Solid
{

private:
    long long int getTime()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return ms.count();
    }

public:
    Eigen::Vector3d pos, eMinusPos; // eMinusPos is computed once per frame
    double C;                       // C is computed once per frame
    double radius;
    bool first = true; // set to false after the first intersection test

    // constructoring using initializer list for parent class and this class
    Sphere(ObjProps props, double x, double y, double z, double r) : Solid(props), pos(Eigen::Vector3d{x, y, z}), radius(r)
    {
    }

    Sphere(ObjProps props, Eigen::Vector3d pos, double radius) : Solid(props), pos(pos), radius(radius)
    {
    }


    //Based on Prof Bartiel's hide.cpp code
    double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, HitRecord &hr)
    
    {
        
        double ddotemc = rayDir.dot(e - pos);
        double d2 = rayDir.squaredNorm();

        double disc = (ddotemc * ddotemc) - d2 * ((e - pos).squaredNorm() - radius * radius);

        if (disc < 0)
            return maxT;
        double root1 = (-ddotemc + std::sqrt(disc)) / d2;
        double root2 = (-ddotemc - std::sqrt(disc)) / d2;

        //this is 1 if we get the first intersection (ray came from outside the object)
        //This is -1 if we get the second intersection (ray came from inside the object, such as if the scene was enclosed in a reflective sphere.)
        int normalDirection = 1; 
        //printf("s t1 = %f, t2 = %f\n", root1, root2);
        double t = root1;
        if (root1 < 0 || (root2 > minT && root2 < root1)){
            t = root2;
            normalDirection = -1;
        }
        if (t < minT || t > maxT)
            return maxT;
        
        //hr.t = t;
        //if(minT == .011)
        //printf("sphere t = %f\n", t);
        // hr.impactPos = e + t * rayDir;
        // hr.normal = (hr.impactPos - pos) / radius;
       
        Eigen::Vector3d normal = (e + t * rayDir - pos) / radius;
        updateHitRecord(t, rayDir, e,  normal, hr);

        // if (rand() % 100 < 2)
        // {
        //     printf("s t1 = %f\n", t);
        //     printVec(hr.normal);
        // }

        return t;
    }

    virtual Eigen::Vector3d getNormal(Eigen::Vector3d & impactPos){
        return (impactPos - pos).normalized();
    }

    //My code from proj1. Calculated t values or normals weren't quite right and I got 
    //fed up with debugging it so I just switched to the prof's code
    // mostly from notes and around page 78 of textbook
    // double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, HitRecord &hr)
    // {
    //     // long long int tic = getTime();
    //     //  First calculate discriminate

    //     double A, B;
    //     // if (first)
    //     // { // precalculate stuff
    //     Eigen::Vector3d eMinusPos = e - pos;
    //     double C = (eMinusPos).dot(eMinusPos) - radius * radius;

    //     // first = false;
    //     // }else{
    //     //     printf("not first error\n");
    //     // }

    //     // A and B need to be calculated for each pixel since rayDir changes.
    //     A = rayDir.squaredNorm(); // rayDir.dot(rayDir);
    //     // long long int toc = getTime();
    //     // TIME += (toc-tic);
    //     // 3992 ms here
    //     B = 2.0 * rayDir.dot(eMinusPos);

    //     // 9286 here
    //     double discriminant = B * B - 4 * A * C;

    //     // 9310 ms here
    //     if (discriminant < 0)
    //         return maxT;

    //     double b = B / 2; // rayDir.dot(eMinusPos);
    //     double root = sqrt(discriminant);

    //     double denom = A; // rayDir.dot(rayDir);

    //     // t2 is the first intersection because t2<t1, and therefore closer to camera.
    //     double t1 = (-b + root) / denom; // second intersection
    //     double t2 = (-b - root) / denom; // first intersection
    //     // printf("t1 = %f, t2 = %f\n", t1, t2);

    //     double t_;

    //     // check the t values
    //     if (t2 > minT)
    //         t_ = t2;
    //     else if (t1 > minT)
    //         t_ = t1;
    //     else
    //         return maxT;

    //     // if we got a good intersection update the hit record

    //     // Calculate the normal vector
    //     Eigen::Vector3d reflectionPoint = rayDir * t_ + e;
    //     // taken from textbook
    //     Eigen::Vector3d normal = (reflectionPoint - pos) / radius;

    //     updateHitRecord(t_, rayDir, e, normal, hr);

    //     if (rand() % 100 < 2)
    //     {
    //         printf("s t1 = %f, t2 = %f\n", t1, t2);
    //         printVec(hr.normal);
    //     }

    //     return t_;
    // }

    // mostly from notes and around page 78 of textbook
    double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT)
    {
        // long long int tic = getTime();
        //  First calculate discriminate

        double A, B;
        // if (first)
        //{ // precalculate stuff
        Eigen::Vector3d eMinusPos = e - pos;
        double C = (eMinusPos).dot(eMinusPos) - radius * radius;

        // Since we're dealing with relfections and such, we can no longer calculate this just once per frame.
        // Later on I might calculate this once per frame and use it only for 1st intersections, but right now I'm leaving it out.
        // That's why first=false is commented out.
        // first = false;
        // }else{
        //     printf("not first error\n");
        // }

        // A and B need to be calculated for each pixel since rayDir changes.
        A = rayDir.squaredNorm(); // rayDir.dot(rayDir);
        // long long int toc = getTime();
        // TIME += (toc-tic);
        // 3992 ms here
        B = 2.0 * rayDir.dot(eMinusPos);

        // 9286 here
        double discriminant = B * B - 4 * A * C;

        // 9310 ms here
        if (discriminant < 0)
            return maxT;

        double b = B / 2; // rayDir.dot(eMinusPos);
        double root = sqrt(discriminant);

        double denom = A; // rayDir.dot(rayDir);

        // t2 is the first intersection because t2<t1, and therefore closer to camera.
        double t1 = (-b + root) / denom; // second intersection
        double t2 = (-b - root) / denom; // first intersection

        // check the t values
        if (t2 > minT)
            return t2;
        else if (t1 > minT)
            return t1;
        else
            return maxT;
    }

    //-------------------------------------------------------------------------------------
    // EXPERIMENTAL BELOW (included in project 1)

    // Checks if this solid is at all inside a node of an octree.
    // Adapted from Graphics Gems 1, "A Simple Method For Box-Sphere Intersection Testing", p. 335.
    bool insideOctree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d center)
    {
        double radSquared = radius * radius;
        double minimumDistance = 0; //

        // loop through every dimension
        for (int dim = 0; dim < 3; dim++)
        {
            if (corner1[dim] > pos[dim])
            {
                minimumDistance += (pos[dim] - corner1[dim]) * (pos[dim] - corner1[dim]);
            }
            else if (corner2[dim] < pos[dim])
            {
                minimumDistance += (pos[dim] - corner2[dim]) * (pos[dim] - corner2[dim]);
            }
        }

        if (minimumDistance >= radSquared)
            return false;
        return true;
    }

    // tests if the solid is entirely contained within the bounding sphere. Turns out this is very slow.
    // bool insideOctree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d center){
    //     double size = std::abs(corner1[0] - corner2[0]) * 2;
    //     double radiusOfBoundingSphere = (size * sqrt(3) / 2.0);
    //     //printf("s size = %f, rad = %f\n", size, radiusOfBoundingSphere);
    //     //check if containment is possible
    //     if(radiusOfBoundingSphere < radius)
    //         return false;

    //     Eigen::Vector3d boundingSphereCenter = center;
    //     //std::cout << "center : " << boundingSphereCenter << std::endl;
    //     Eigen::Vector3d diff = boundingSphereCenter - pos;
    //     return diff.norm() <= (radiusOfBoundingSphere - radius);
    // }

    // IGNORE BELOW CODE FOR PROJECT 1

    // this function is garbage and doesn't work at all. Keeping it around only so I can reference to some Eigen syntax.
    Eigen::Vector4d raytrace(Eigen::Vector3d rayDir, Eigen::Vector3d e,
                             double minT, double maxT, int reflectionsRemaining, std::vector<Solid *> &solids, Solid *parent)
    {
        // First calculate discriminate
        Eigen::Vector3d color_ = properties.color;

        Eigen::Vector4d color;
        // color << color_, Eigen::VectorXd{0};
        color << color_[0], color_[1], color_[2], 0;

        double A, B, C;
        Eigen::Vector3d eMinusPos = e - pos;

        A = rayDir.dot(rayDir);
        B = 2 * rayDir.dot(eMinusPos);
        C = (eMinusPos).dot(eMinusPos) - radius * radius;

        double discriminant = B * B - 4 * A * C;

        // if the ray didn't hit anything return black (for now)
        if (discriminant < 0)
            return Eigen::Vector4d{0, 0, 0, maxT};

        double b = rayDir.dot(eMinusPos);
        double root = sqrt(discriminant);

        double denom = A; // rayDir.dot(rayDir);

        // t2 is the first intersection because t2<t1, and therefore closer to camera.

        double t1 = (-b + root) / denom; // second intersection
        double t2 = (-b - root) / denom; // first intersection
        // printf("t1 = %f, t2 = %f\n", t1, t2);

        if (reflectionsRemaining == 0)
        {
            // std::cout << t2 << std::endl;
            if (t2 > 0)
                color[3] = t2; // maybe dont overwrite here
            else if (t1 > 0)
                color[3] = t1;
            else
                color[3] = -1;
            return color;
        }
        else
        {
            if (t2 > 0)
                color[3] = t2; // maybe dont overwrite here
            else if (t1 > 0)
                color[3] = t1;

            Eigen::Vector3d reflectionPoint = rayDir * t1 + e;

            if (t2 > 0)
                reflectionPoint = rayDir * t2 + e;

            // Eigen::Vector3d unitNormalToHit = (reflectionPoint - pos) / radius;
            Eigen::Vector3d normal = 2 * (reflectionPoint - pos);
            Eigen::Vector3d reflectionRay = rayDir - 2 * rayDir.dot(normal) * normal;
            // reflectionRay *= -1;

            int maxReflections = reflectionsRemaining - 1;
            double maxT = 10000000000;
            double minT = 0;
            double intersectionT = maxT;
            Solid *closest = nullptr;
            Eigen::Vector4d reflectionColor{0, 0, 0, maxT};
            for (Solid *s : solids)
            {
                // dont ray trace against itself
                if (s != this)
                {
                    Eigen::Vector4d possibleColor = s->raytrace(reflectionRay, reflectionPoint, minT, intersectionT, maxReflections, solids, nullptr);

                    if (reflectionColor[3] > possibleColor[3] && possibleColor[3] > 0)
                    {
                        // std::cout << "Reflection!\n";
                        intersectionT = possibleColor[3];
                        closest = s;
                        reflectionColor = possibleColor;
                    }
                }
            }
            float totalT = color[3];
            //  if(reflectionColor[3] != maxT)
            //      totalT += reflectionColor[3];

            float ap = .5;
            float bp = 1 - ap;
            color = color * ap + reflectionColor * bp;

            color[3] = totalT;
            // if(t2 > 0)
            //     color[3] = t2; // maybe dont overwrite here
            // else if(t1 > 0)
            //     color[3] = t1;
            // else
            //     color[3] = -1;
            return color;
        }
        // if(t2 > minT)
        //     return t2;
        // else if(t1 > minT)
        //     return t1;
        // else
        //     return maxT;
    }

    // //TODO (maybe): change how this works. Right now it looks bad.
    // std::string toString(){
    //     return "Position:\n" + Entity::toString() + "\n Color: " +  to_string(r) + " " + to_string(g) + " " + to_string(b) + "\n";
    // }

    string TYPE()
    {
        return "Sphere";
    }

    int type(){
        return 1;
    }
};

#endif
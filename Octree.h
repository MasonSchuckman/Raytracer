#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "Solid.h"
#include <stack>
#include <string>
#include "Sphere.h"
#include <bits/stdc++.h>
#include <type_traits>

#ifndef Octree_H
#define Octree_H

// code adapted from a Java Octree class I wrote in highschool
template <typename T>
class Octree
{
private:
    long long int getTime()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return ms.count();
    }

public:
    // The smallest sphere that can enclose the octant that this node defines.
    // Using this so I can check if a Ray (probably) intersects this node. A better way would be to actually check if a ray
    // intersects the cube this node defines, but its much easier to just do spheres since I already programmed that
    Sphere *boundingSphere;
    Octree *parent;
    std::vector<Octree *> children;
    int obsContained = 0; // how many solids are in this octant (or its children (recursive))

    double MIN_SIZE = .01;   // the minimum side length an octant must be larger than to qualify for children
    int minObs = 5;          // the minimum number of objects in an octant to justify making a new octree node
    double size;             // side length of this octant
    double distFromCam = -1; // will be set properly by a function later
    std::vector<Eigen::Vector3d *> octantBounds;
    //  octants go in this order(if looking at a cube face on):
    /*
     * far away:
     * 	 north west, north east		|0,1|
     *   south west, south east		|2,3|
     * close side:
     * 								|4,5|
     * 								|6,7|
     */
    Eigen::Vector3d origin;

    std::unordered_set<T> octList[8];
    std::unordered_set<T> solids;
    std::unordered_set<T> allChildObs;
    // ArrayList <Freebody> [] octList=new ArrayList [8];//contains all the objects that fit within each respective octant.
    // ArrayList <Freebody> obs;

    Octree(Octree *_parent, Eigen::Vector3d *bounds, const std::unordered_set<T> &objectsInOctant, int &counter)
        : Octree(_parent, bounds[0], bounds[1][0] - bounds[0][0], objectsInOctant, counter)
    {
    }

    Octree(Octree *_parent, Eigen::Vector3d roots, double size_, const std::unordered_set<T> &objectsInOctant, int &counter)
    {
        counter++; // counts how many Octree nodes we've created
        // printf("Light oc size = %d\n", objectsInOctant.size());
        //  printf("\nSize = %f num obs = %d\n", size_, objectsInOctant.size());
        //  set some variables
        size = size_;
        // solids = objectsInOctant;
        parent = _parent;

        // origin is the center of the octant/octree
        origin[0] = roots[0] + size / 2;
        origin[1] = roots[1] + size / 2;
        origin[2] = roots[2] + size / 2;

        // calculate the radius of the bounding sphere
        double radiusOfBoundingSphere = (size * sqrt(3) / 2.0);

        // The boundingSphere for an octant is how we check for intersection of a ray with an octant.
        // While this overestimates the octant's volume and hitbox, that's okay.
        boundingSphere = new Sphere(ObjProps(), origin, radiusOfBoundingSphere);

        for (int i = 0; i < 8; i++)
        {
            octList[i] = std::unordered_set<T>();
        }
        // octantBounds = new Eigen::Vector3d *[8];
        octantBounds.reserve(8);
        setupOctantBounds();

        std::unordered_set<T> rem(objectsInOctant.size() / 2);

        // sort the objects in this octant into the child octants.
        // Make sure this octant is able to have children.
        if (size / 2 > MIN_SIZE && objectsInOctant.size() > minObs)
            for (T s : objectsInOctant)
            {
                for (int i = 0; i < 8; i++)
                {
                    // Get the bounding vectors for the current child
                    Eigen::Vector3d childCorner1 = octantBounds[i][0];
                    Eigen::Vector3d childCorner2 = octantBounds[i][1];
                    // printf("o size = %f, rad = %f\n", size, radiusOfBoundingSphere);
                    // std::cout << "o center : " << origin  << std::endl;

                    // Mini specialization for checking if a Solid is inside the octant vs if a vector is inside the octant

                    if (insideOctree(s, childCorner1, childCorner2, origin))
                    {
                        // One object might be in two octants, but the object can only be removed the parent's ArrayList once.
                        // That's why I use a set here. Also its fast.

                        rem.insert(s);
                        octList[i].insert(s);
                    }
                }
            }

        // preallocate space
        children.reserve(8);

        // creating the children octrees once objects have been determined to lie within them
        for (int i = 0; i < 8; i++)
        {
            // printf("Octant %d has %d obs in it\n", i, octList[i].size());
            // only create a child if it will have any solids in it
            if (octList[i].size() > 0)
            {
                // std::cout << "Created child node. obs in child = " << octList[i].size() << std::endl;
                children[i] = new Octree<T>(this, octantBounds[i][0], octantBounds[i][1][0] - octantBounds[i][0][0], octList[i], counter);
            }
            else
            {
                children[i] = nullptr;
            }
        }

        // long long int t1 = getTime();

        // remove all the solids that were placed into children octants from this octant.
        //(if a solid is in a child, no reason to store it in this octant)
        // Using a set for the removal data structure as well as the solids data structure was by far the fastest
        // configuration I found.
        allChildObs = objectsInOctant;

        if (true)
        // if (rem.size() > .5 * objectsInOctant.size())
        {
            for (auto it = objectsInOctant.begin(); it != objectsInOctant.end(); it++)
            {
                if (rem.find(*it) == rem.end())
                    solids.insert(*it);
            }
        }
        else
        {
            solids = objectsInOctant;
            for (auto it = rem.begin(); it != rem.end(); it++)
            {
                solids.erase(*it);
            }
        }

        // The code commented out below code is for validation and testing optimizations. You can ignore it.

        // long long int t2 = getTime();
        // counter += (t2 - t1);
        // make sure we didn't mistakenly discard any solids or duplicate something
        // std::unordered_set<T> total;
        // validateObjectCount(total);
        // obsContained = total.size();
        // if (total.size() != objectsInOctant.size())
        // {
        //     printf("something went wrong %d\n", (total.size() - objectsInOctant.size()));
        // }

        // if (total.size() == 0)
        // {
        //     printf("This octant is useless\n");
        // }

        // std::unordered_set<T> childrenObs;
        // getObsInChildren(childrenObs);
        // for (T s : childrenObs)
        //     allChildObs.insert(s);

        // std::cout << "Finished constructor. obs in octant = " << solids.size() << std::endl;
    }

    bool insideOctree(T, Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d origin)
    {
        printf("default inside octree\n");
        return false;
    }

    template <typename Q>
    bool insideOctree(T, Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d origin)
    {
        printf("default inside octree\n");
        return false;
    }

    double getIntensity(double maxDist, Eigen::Vector3d pos, int &c);
    template <typename X>
    double getIntensity(double maxDist, Eigen::Vector3d pos, int &c); // { printf("default 1\n"); return 0; }

    // template <typename Q>
    // double getIntensity(double maxDist, Eigen::Vector3d pos, int& c) { printf("default 2\n"); return 0; }

    // Function to test runtime optimizations.
    // When called at the root of the Octree, it will return the number of octrees that
    // theoritically could be replace by a child node, without disturbing any data.
    // Only true if this octant if
    //   a) Doesn't have any solids in it
    //   AND
    //   b) Only has 1 child octant with solids in it
    int countReplaceable()
    {
        int count = 0;
        int numChildren = 0;
        bool replaceable = true;
        if (solids.size() > 0)
            replaceable = false;

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                numChildren++;
                count += children[i]->countReplaceable();
            }
        }

        if (numChildren < 2 && solids.size() == 0)
            replaceable = true;
        if (replaceable)
            return count + 1;
        else
            return count;
    }

    // makes sure that the number of objects in this octant plus the number of objects in all
    // children (recursive) equals the number of objects that was originally passed to this Octree node.
    void validateObjectCount(std::unordered_set<T> &tot)
    {
        for (T s : solids)
            tot.insert(s);

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                children[i]->validateObjectCount(tot);
            }
        }
    }

    // Populates tot with all the children in this octant or its children (recursive)
    void getObsInChildren(std::unordered_set<T> &tot)
    {
        for (std::unordered_set<T> childrenObs : octList)
            for (T s : childrenObs)
                tot.insert(s);

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                children[i]->getObsInChildren(tot);
            }
        }
    }

    // used to sort octants based on distance from camera. Not sure it actually works.
    struct comparator
    {
        inline bool operator()(Octree *a, Octree *b)
        {
            return (a->distFromCam < b->distFromCam);
        }
    };

    // calculates the minimum distance a Solid is in this octant (or its children) to the camera location.
    // Used for runtime optimization
    double calcMinDistanceFromCam(Eigen::Vector3d &e)
    {
        // if (distFromCam == -1)
        // {
        //     // std::cout << "here" << std::endl;
        //     double dist = 10000000000;
        //     for (Ts : solids)
        //         dist = std::min(dist, s->calcDistanceToCam(e));

        //     for (int i = 0; i < 8; i++)
        //     {
        //         if (children[i] != nullptr)
        //         {
        //             dist = std::min(dist, children[i]->calcMinDistanceFromCam(e));
        //         }
        //     }

        //     distFromCam = dist;

        //     // sort this octant's children by dist to camera
        //     // std::sort(children.begin(), children.end(), comparator());
        //     // for (Octree *o : children)
        //     // {
        //     //     printf("dist = %f\n", o->distFromCam);
        //     // }
        // }
        // return distFromCam;
        return -1; //"Turning off" this function for now since I suspect it may be breaking reflections since the "from" is changing.
    }

    // PROJ2 intersect function
    void intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, double _nearest, HitRecord &hr)
    {

        // counter++;
        // if (distFromCam == -1)
        // {
        //     //distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        if (boundingSphere->intersect(rayDir, e, minT, maxT) >= maxT)
        {
            return;
        }

        // setup vars for the intersection tests
        double nearest = hr.t;
        double intersectionS = maxT;
        T closest = hr.nearest;

        // long long int t1 = getTime();

        // first check the solids that are directly in this octant
        for (T s : solids)
        {
            if (s->distToCam < nearest)
            {

                intersectionS = s->intersect(rayDir, e, minT, maxT, hr);

                if (nearest > intersectionS)
                {
                    nearest = intersectionS;
                    closest = s;
                }
                // triCounter++;
            }
            else
            {
                printf("solid error\n");
            }
        }
        // long long int t2 = getTime();
        // timer += (t2 - t1);

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr) // make sure the child exists
            {
                if (nearest > children[i]->distFromCam) // check if its possible that this child yeilds a closer intersection that what we already have
                {
                    children[i]->intersect(rayDir, e, minT, maxT, nearest, hr);
                    // intersectionS = possibleInfo.t;
                    // if (nearest > intersectionS)
                    // {
                    //     nearest = intersectionS;
                    //     closest = possibleInfo.nearest;
                    // }
                }
                else
                {
                    printf("octree error. Nearest = %f, dist = %f\n", nearest, children[i]->distFromCam);
                }
            }
        }
    }

    bool checkOccluded(T parentSolid, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT)
    {

        // if (distFromCam == -1)
        // {
        //     distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        // if (boundingSphere->radius > maxT / 2)
        //{
        if (boundingSphere->intersect(rayDir, e, minT, 10000000) >= 10000000) // WARNING!! This might cause issues since now we're checking against dist to light!!
            return false;
        //}
        // else if (boundingSphere->intersect(rayDir, e, minT, maxT * 2) >= maxT * 2) // WARNING!! This might cause issues since now we're checking against dist to light!!
        //    return false;

        // first check the solids that are directly in this octant
        for (T s : solids)
        {
            if (parentSolid != s && s->intersect(rayDir, e, minT, maxT) != maxT)
                return true;
        }

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            // If the child exists check if any solids within it occlude the surface
            if (children[i] != nullptr && children[i]->checkOccluded(parentSolid, rayDir, e, minT, maxT))
                return true;
        }

        return false;
    }

    bool checkOccluded2(T parentSolid, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, Light &l, double &refractionCoef)
    {

        // if (distFromCam == -1)
        // {
        //     distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        // if (boundingSphere->radius > maxT / 2)
        //{
        if (boundingSphere->intersect(rayDir, e, minT, 10000000) >= 10000000) // WARNING!! This might cause issues since now we're checking against dist to light!!
            return false;
        //}
        // else if (boundingSphere->intersect(rayDir, e, minT, maxT * 2) >= maxT * 2) // WARNING!! This might cause issues since now we're checking against dist to light!!
        //    return false;
        HitRecord hr;
        // first check the solids that are directly in this octant
        for (T s : solids)
        {
            if (parentSolid != s && s->intersect(rayDir, e, minT, maxT, hr) != maxT)
                // if the solid is transparent we need to check if the refraction ray has LOS to the light.
                if (s->properties.T > 0)
                {

                    // declare vars
                    double ior = s->properties.index_of_refraction;
                    Eigen::Vector3d normal = hr.normal;
                    Eigen::Vector3d viewDir = -hr.rayDir; // already normalized
                    double dot = normal.dot(viewDir);

                    // We need the normal to the surface at the point of impact to know whether we're entering or exiting the surface.
                    // If our rayDir is aligned with the normal, then we're exiting the surface, otherwise we're entering.
                    // Luckily we already computed this when updating the hit record.

                    // check if the ray is entering or exiting the surface and set the index of refraction accordingly.
                    // Since we're setting dot = normal dot viewDir instead of normal dot rayDir, our rayDir is aligned with normal if the
                    // dotproduct is positive instead of negative
                    bool entering = (dot >= 0);
                    if (entering)
                    {
                        ior = 1.0 / ior;
                    }
                    else
                    {
                        // If we're exitting the surface we need to negate the normal vector and thus the dot product.
                        normal = -normal;
                        dot = -dot;
                    }

                    // setup vars for occlusion test
                    double maxT = (l.pos - hr.impactPos).norm();
                    double minT = minT;

                    // We need to calculate the discriminate first so we don't try to take the sqrt of a negative.
                    double discriminate = 1 - (1 - (dot * dot)) * (ior * ior);
                    if (discriminate > 0)
                    {
                        // calc refraction ray dir
                        Eigen::Vector3d refractionDir = (normal * (dot)-viewDir) * ior - normal * std::sqrt(discriminate);

                        // Check occlusion for refraction ray
                        if (checkOccluded2(nullptr, refractionDir, hr.impactPos, minT, maxT, l, refractionCoef))
                            return true;
                    }
                    // only checking refraction right now
                    //  else
                    //  {
                    //      Eigen::Vector3d internalReflectionDir = (hr.rayDir + 2.0 * -hr.rayDir.dot(normal) * normal).normalized();

                    //     // Check occlusion for reflection ray
                    //     if (checkOccluded2(s, internalReflectionDir, hr.impactPos, minT, maxT, l))
                    //         return true;
                    // }
                    // double t = s->properties.T;
                    // double ior = s->properties.index_of_refraction;
                    // double rad = ((Sphere*)s)->radius;
                    // refractionCoef *= t;
                    // double efl =  ior * rad / (2 * (ior - 1));
                    // double bfl = efl - rad;
                    // refractionCoef *= (1 - )
                }
                else
                    return true;
        }

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            // If the child exists check if any solids within it occlude the surface
            if (children[i] != nullptr && children[i]->checkOccluded2(parentSolid, rayDir, e, minT, maxT, l, refractionCoef))
                return true;
        }

        return false;
    }

    // Accounts for refracting objects
    // If the only objects occluding this light are transparent, we will cast rays from the light at this position and see if we get close.
    bool checkOccluded3(T parentSolid, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, bool &onlyTransparentOccluding, T &closest)
    {
        bool anything = false;
        // if (distFromCam == -1)
        // {
        //     distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        // if (boundingSphere->radius > maxT / 2)
        //{
        if (boundingSphere->intersect(rayDir, e, minT, 10000000) >= 10000000) // WARNING!! This might cause issues since now we're checking against dist to light!!
            return false;
        //}
        // else if (boundingSphere->intersect(rayDir, e, minT, maxT * 2) >= maxT * 2) // WARNING!! This might cause issues since now we're checking against dist to light!!
        //    return false;
        HitRecord hr;
        hr.t = maxT;
        hr.nearest = nullptr;
        double t = maxT;
        double furthest = minT;
        // first check the solids that are directly in this octant
        for (T s : solids)
        {
            // printf("pre check\n");
            if (parentSolid != s)
            {
            }
            t = s->intersect(rayDir, e, minT, maxT, hr);
            if (t < maxT)
            {
                anything = true;
                if (t > furthest)
                {
                    furthest = t; // dist of object from surface
                    closest = s;  // transparent object closest to light
                }
                // if the solid is transparent we need to check if the refraction ray has LOS to the light.
                if (s->properties.T == 0)
                {
                    std::cout << "type = " << s->TYPE() << std::endl;
                    std::cout << "parent type = " << parentSolid->TYPE() << std::endl;
                    printf("Verts:\n");
                    ((Solid *)s)->printVec(((Solid *)s)->verts[0]);
                    ((Solid *)s)->printVec(((Solid *)s)->verts[1]);
                    ((Solid *)s)->printVec(((Solid *)s)->verts[2]);
                    ((Solid *)s)->printVec(e);
                    ((Solid *)s)->printVec(hr.impactPos);
                    ((Solid *)s)->printVec(rayDir);
                    printf("t = %f, maxT = %f, minT = %f\n", t, maxT, minT);
                    onlyTransparentOccluding = false;
                    return true;
                }
                else if (s->TYPE() != "Sphere")
                { // type check here since I've only really tested spheres
                    // printf("got here\n");
                    onlyTransparentOccluding = false;
                    return true;
                }
            }
        }

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            // If the child exists check if any solids within it occlude the surface
            if (children[i] != nullptr && children[i]->checkOccluded3(parentSolid, rayDir, e, minT, maxT, onlyTransparentOccluding, closest))
                return true;
        }

        return anything;
    }

    ~Octree()
    {
        // std::cout << "Destructor" << std::endl;

        for (int i = 0; i < 8; i++)
        {
            delete[] octantBounds[i];

            if (children[i] != nullptr)
                delete children[i];
        }

        delete boundingSphere;
    }

    // helper function for populating the Octrees' bounding vectors
    Eigen::Vector3d *makeBounds(double x, double y, double z, double size)
    { // makes the bounds based on origin x,y and the size
        Eigen::Vector3d *bounds = new Eigen::Vector3d[2];

        Eigen::Vector3d frontTopRight, backBottomLeft;

        // int[][] b = new int[3][2];
        frontTopRight << x, y, z;
        backBottomLeft << x + size, y + size, z + size;
        bounds[0] = frontTopRight;
        bounds[1] = backBottomLeft;

        return bounds;
    }

    // Populates the Octrees' bounding vectors
    void setupOctantBounds()
    {
        // std::cout << "Setting up octant bounds" << std::endl;

        double x = origin[0];
        double y = origin[1];
        double z = origin[2];

        // far octants first
        octantBounds[0] = makeBounds(x - size / 2, y - size / 2, z - size / 2, size / 2); // NW
        octantBounds[1] = makeBounds(x, y - size / 2, z - size / 2, size / 2);            // NE

        octantBounds[2] = makeBounds(x - size / 2, y, z - size / 2, size / 2); // SW
        octantBounds[3] = makeBounds(x, y, z - size / 2, size / 2);            // SE

        // close octants now (changing z)
        octantBounds[4] = makeBounds(x - size / 2, y - size / 2, z, size / 2); // NW
        octantBounds[5] = makeBounds(x, y - size / 2, z, size / 2);            // NE

        octantBounds[6] = makeBounds(x - size / 2, y, z, size / 2); // SW
        octantBounds[7] = makeBounds(x, y, z, size / 2);            // SE
        // std::cout << "Finished setting up octant bounds" << std::endl;
    }

    std::string toString(int spacer)
    {
        // std::cout << "in tostring1 " << std::endl;

        std::string data = "";
        std::string space = "";
        for (int i = 0; i < spacer * 4; i++)
        {
            space += " ";
        }
        if (solids.size() > 0)
            data += space + "Level " + std::to_string(spacer) + ", Size = " + std::to_string(size) + " with " + std::to_string(solids.size()) + " objects\n" + space + "\n";
        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                data += children[i]->toString(spacer + 1);
            }
        }
        // std::cout << "in tostring " << std::endl;

        return data;
    }

    // the rest of this file contains misc code I don't want to delete yet.

    // code to print info about this octant and its children.
    //  if (parent == nullptr)
    //  {
    //      for (int child = 0; child < 8; child++)
    //      {
    //          if (children[child] != nullptr)
    //          {
    //              printf("child = %d\n", child);
    //              for (int oc = 0; oc < 8; oc++)
    //              {
    //                  printf("Octant = %d\n", oc);

    //                 for (int j = 0; j < 3; j++)
    //                 {
    //                     for (int i = 0; i < 2; i++)
    //                     {
    //                         printf("%.0f ", children[child]->octantBounds[oc][i][j]);
    //                     }
    //                     printf("\n");
    //                 }
    //             }
    //         }
    //     }
    // }

    // Eigen::Vector3d corner1, corner2; //eMinusPos is computed once per frame
    // std::vector<Octree*> children;
    // std::vector<Solid*> objects;
    // double size;
    // double C; //C is computed once per frame
    // double radius;
    // bool first = true;

    // Octree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, std::vector<Solid*> solids, int minSize) : corner1(corner1), corner2(corner2)
    // {
    //     size = (corner1 - corner2).norm();

    //     //if this node is already at the minimum size store the rest of solids in this node.
    //     if(size < minSize)
    //         objects = solids;
    // }

    // void buildOctree(std::vector<Solid*> solids, int minSize){
    //     children = std::vector<Octree*>(8);
    //     std::vector<std::vector<Solid*>> childNodeSolids;

    //     for(int i = 0; i < 8; i++){
    //         Eigen::Vector3d childCorner1;
    //         Eigen::Vector3d childCorner2;

    //         for(Solid* s : solids){
    //             if(s->insideOctree(childCorner1, childCorner2)){
    //                 childNodeSolids[i].push_back(s);
    //             }
    //         }
    //     }
    // }
};

template <>
class Octree<Solid *>
{
private:
    long long int getTime()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return ms.count();
    }

public:
    // The smallest sphere that can enclose the octant that this node defines.
    // Using this so I can check if a Ray (probably) intersects this node. A better way would be to actually check if a ray
    // intersects the cube this node defines, but its much easier to just do spheres since I already programmed that
    Sphere *boundingSphere;
    Octree *parent;
    std::vector<Octree *> children;
    int obsContained = 0; // how many solids are in this octant (or its children (recursive))

    double MIN_SIZE = .01;   // the minimum side length an octant must be larger than to qualify for children
    int minObs = 5;          // the minimum number of objects in an octant to justify making a new octree node
    double size;             // side length of this octant
    double distFromCam = -1; // will be set properly by a function later
    std::vector<Eigen::Vector3d *> octantBounds;
    //  octants go in this order(if looking at a cube face on):
    /*
     * far away:
     * 	 north west, north east		|0,1|
     *   south west, south east		|2,3|
     * close side:
     * 								|4,5|
     * 								|6,7|
     */
    Eigen::Vector3d origin;

    std::unordered_set<Solid *> octList[8];
    std::unordered_set<Solid *> solids;
    std::unordered_set<Solid *> allChildObs;
    // ArrayList <Freebody> [] octList=new ArrayList [8];//contains all the objects that fit within each respective octant.
    // ArrayList <Freebody> obs;

    Octree(Octree *_parent, Eigen::Vector3d *bounds, const std::unordered_set<Solid *> &objectsInOctant, int &counter)
        : Octree(_parent, bounds[0], bounds[1][0] - bounds[0][0], objectsInOctant, counter)
    {
    }

    Octree(Octree *_parent, Eigen::Vector3d roots, double size_, const std::unordered_set<Solid *> &objectsInOctant, int &counter)
    {
        counter++; // counts how many Octree nodes we've created
        // printf("Light oc size = %d\n", objectsInOctant.size());
        //  printf("\nSize = %f num obs = %d\n", size_, objectsInOctant.size());
        //  set some variables
        size = size_;
        // solids = objectsInOctant;
        parent = _parent;

        // origin is the center of the octant/octree
        origin[0] = roots[0] + size / 2;
        origin[1] = roots[1] + size / 2;
        origin[2] = roots[2] + size / 2;

        // calculate the radius of the bounding sphere
        double radiusOfBoundingSphere = (size * sqrt(3) / 2.0);

        // The boundingSphere for an octant is how we check for intersection of a ray with an octant.
        // While this overestimates the octant's volume and hitbox, that's okay.
        boundingSphere = new Sphere(ObjProps(), origin, radiusOfBoundingSphere);

        for (int i = 0; i < 8; i++)
        {
            octList[i] = std::unordered_set<Solid *>();
        }
        // octantBounds = new Eigen::Vector3d *[8];
        octantBounds.reserve(8);
        setupOctantBounds();

        std::unordered_set<Solid *> rem(objectsInOctant.size() / 2);

        // sort the objects in this octant into the child octants.
        // Make sure this octant is able to have children.
        if (size / 2 > MIN_SIZE && objectsInOctant.size() > minObs)
            for (Solid *s : objectsInOctant)
            {
                for (int i = 0; i < 8; i++)
                {
                    // Get the bounding vectors for the current child
                    Eigen::Vector3d childCorner1 = octantBounds[i][0];
                    Eigen::Vector3d childCorner2 = octantBounds[i][1];
                    // printf("o size = %f, rad = %f\n", size, radiusOfBoundingSphere);
                    // std::cout << "o center : " << origin  << std::endl;

                    // Mini specialization for checking if a Solid is inside the octant vs if a vector is inside the octant

                    if (insideOctree(s, childCorner1, childCorner2, origin))
                    {
                        // One object might be in two octants, but the object can only be removed the parent's ArrayList once.
                        // That's why I use a set here. Also its fast.

                        rem.insert(s);
                        octList[i].insert(s);
                    }
                }
            }

        // preallocate space
        children.reserve(8);

        // creating the children octrees once objects have been determined to lie within them
        for (int i = 0; i < 8; i++)
        {
            // printf("Octant %d has %d obs in it\n", i, octList[i].size());
            // only create a child if it will have any solids in it
            if (octList[i].size() > 0)
            {
                // std::cout << "Created child node. obs in child = " << octList[i].size() << std::endl;
                children[i] = new Octree<Solid *>(this, octantBounds[i][0], octantBounds[i][1][0] - octantBounds[i][0][0], octList[i], counter);
            }
            else
            {
                children[i] = nullptr;
            }
        }

        // long long int t1 = getTime();

        // remove all the solids that were placed into children octants from this octant.
        //(if a solid is in a child, no reason to store it in this octant)
        // Using a set for the removal data structure as well as the solids data structure was by far the fastest
        // configuration I found.
        allChildObs = objectsInOctant;

        if (true)
        // if (rem.size() > .5 * objectsInOctant.size())
        {
            for (auto it = objectsInOctant.begin(); it != objectsInOctant.end(); it++)
            {
                if (rem.find(*it) == rem.end())
                    solids.insert(*it);
            }
        }
        else
        {
            solids = objectsInOctant;
            for (auto it = rem.begin(); it != rem.end(); it++)
            {
                solids.erase(*it);
            }
        }

        // The code commented out below code is for validation and testing optimizations. You can ignore it.

        // long long int t2 = getTime();
        // counter += (t2 - t1);
        // make sure we didn't mistakenly discard any solids or duplicate something
        // std::unordered_set<T> total;
        // validateObjectCount(total);
        // obsContained = total.size();
        // if (total.size() != objectsInOctant.size())
        // {
        //     printf("something went wrong %d\n", (total.size() - objectsInOctant.size()));
        // }

        // if (total.size() == 0)
        // {
        //     printf("This octant is useless\n");
        // }

        // std::unordered_set<T> childrenObs;
        // getObsInChildren(childrenObs);
        // for (T s : childrenObs)
        //     allChildObs.insert(s);

        // std::cout << "Finished constructor. obs in octant = " << solids.size() << std::endl;
    }

    // Function to test runtime optimizations.
    // When called at the root of the Octree, it will return the number of octrees that
    // theoritically could be replace by a child node, without disturbing any data.
    // Only true if this octant if
    //   a) Doesn't have any solids in it
    //   AND
    //   b) Only has 1 child octant with solids in it
    int countReplaceable()
    {
        int count = 0;
        int numChildren = 0;
        bool replaceable = true;
        if (solids.size() > 0)
            replaceable = false;

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                numChildren++;
                count += children[i]->countReplaceable();
            }
        }

        if (numChildren < 2 && solids.size() == 0)
            replaceable = true;
        if (replaceable)
            return count + 1;
        else
            return count;
    }

    // makes sure that the number of objects in this octant plus the number of objects in all
    // children (recursive) equals the number of objects that was originally passed to this Octree node.
    void validateObjectCount(std::unordered_set<Solid *> &tot)
    {
        for (Solid *s : solids)
            tot.insert(s);

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                children[i]->validateObjectCount(tot);
            }
        }
    }

    bool insideOctree(Solid *s, Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d origin)
    {
        return s->insideOctree(corner1, corner2, origin);
    }

    // Populates tot with all the children in this octant or its children (recursive)
    void getObsInChildren(std::unordered_set<Solid *> &tot)
    {
        for (std::unordered_set<Solid *> childrenObs : octList)
            for (Solid *s : childrenObs)
                tot.insert(s);

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                children[i]->getObsInChildren(tot);
            }
        }
    }

    // used to sort octants based on distance from camera. Not sure it actually works.
    struct comparator
    {
        inline bool operator()(Octree *a, Octree *b)
        {
            return (a->distFromCam < b->distFromCam);
        }
    };

    // calculates the minimum distance a Solid is in this octant (or its children) to the camera location.
    // Used for runtime optimization
    double calcMinDistanceFromCam(Eigen::Vector3d &e)
    {

        return -1; //"Turning off" this function for now since I suspect it may be breaking reflections since the "from" is changing.
    }

    // PROJ2 intersect function
    void intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, double _nearest, HitRecord &hr)
    {

        // counter++;
        // if (distFromCam == -1)
        // {
        //     //distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        if (boundingSphere->intersect(rayDir, e, minT, maxT) >= maxT)
        {
            return;
        }

        // setup vars for the intersection tests
        double nearest = hr.t;
        double intersectionS = maxT;
        Solid *closest = hr.nearest;

        // long long int t1 = getTime();

        // first check the solids that are directly in this octant
        for (Solid *s : solids)
        {
            if (s->distToCam < nearest)
            {

                intersectionS = s->intersect(rayDir, e, minT, maxT, hr);

                if (nearest > intersectionS)
                {
                    nearest = intersectionS;
                    closest = s;
                }
                // triCounter++;
            }
            else
            {
                printf("solid error\n");
            }
        }
        // long long int t2 = getTime();
        // timer += (t2 - t1);

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr) // make sure the child exists
            {
                if (nearest > children[i]->distFromCam) // check if its possible that this child yeilds a closer intersection that what we already have
                {
                    children[i]->intersect(rayDir, e, minT, maxT, nearest, hr);
                    // intersectionS = possibleInfo.t;
                    // if (nearest > intersectionS)
                    // {
                    //     nearest = intersectionS;
                    //     closest = possibleInfo.nearest;
                    // }
                }
                else
                {
                    printf("octree error. Nearest = %f, dist = %f\n", nearest, children[i]->distFromCam);
                }
            }
        }
    }

    bool checkOccluded(Solid *parentSolid, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT)
    {

        // if (distFromCam == -1)
        // {
        //     distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        // if (boundingSphere->radius > maxT / 2)
        //{
        if (boundingSphere->intersect(rayDir, e, minT, 10000000) >= 10000000) // WARNING!! This might cause issues since now we're checking against dist to light!!
            return false;
        //}
        // else if (boundingSphere->intersect(rayDir, e, minT, maxT * 2) >= maxT * 2) // WARNING!! This might cause issues since now we're checking against dist to light!!
        //    return false;

        // first check the solids that are directly in this octant
        for (Solid *s : solids)
        {
            if (parentSolid != s && s->intersect(rayDir, e, minT, maxT) != maxT)
                return true;
        }

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            // If the child exists check if any solids within it occlude the surface
            if (children[i] != nullptr && children[i]->checkOccluded(parentSolid, rayDir, e, minT, maxT))
                return true;
        }

        return false;
    }

    bool checkOccluded2(Solid *parentSolid, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, Light &l, double &refractionCoef)
    {

        // if (distFromCam == -1)
        // {
        //     distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        // if (boundingSphere->radius > maxT / 2)
        //{
        if (boundingSphere->intersect(rayDir, e, minT, 10000000) >= 10000000) // WARNING!! This might cause issues since now we're checking against dist to light!!
            return false;
        //}
        // else if (boundingSphere->intersect(rayDir, e, minT, maxT * 2) >= maxT * 2) // WARNING!! This might cause issues since now we're checking against dist to light!!
        //    return false;
        HitRecord hr;
        // first check the solids that are directly in this octant
        for (Solid *s : solids)
        {
            if (parentSolid != s && s->intersect(rayDir, e, minT, maxT, hr) != maxT)
                // if the solid is transparent we need to check if the refraction ray has LOS to the light.
                if (s->properties.T > 0)
                {

                    // declare vars
                    double ior = s->properties.index_of_refraction;
                    Eigen::Vector3d normal = hr.normal;
                    Eigen::Vector3d viewDir = -hr.rayDir; // already normalized
                    double dot = normal.dot(viewDir);

                    // We need the normal to the surface at the point of impact to know whether we're entering or exiting the surface.
                    // If our rayDir is aligned with the normal, then we're exiting the surface, otherwise we're entering.
                    // Luckily we already computed this when updating the hit record.

                    // check if the ray is entering or exiting the surface and set the index of refraction accordingly.
                    // Since we're setting dot = normal dot viewDir instead of normal dot rayDir, our rayDir is aligned with normal if the
                    // dotproduct is positive instead of negative
                    bool entering = (dot >= 0);
                    if (entering)
                    {
                        ior = 1.0 / ior;
                    }
                    else
                    {
                        // If we're exitting the surface we need to negate the normal vector and thus the dot product.
                        normal = -normal;
                        dot = -dot;
                    }

                    // setup vars for occlusion test
                    double maxT = (l.pos - hr.impactPos).norm();
                    double minT = minT;

                    // We need to calculate the discriminate first so we don't try to take the sqrt of a negative.
                    double discriminate = 1 - (1 - (dot * dot)) * (ior * ior);
                    if (discriminate > 0)
                    {
                        // calc refraction ray dir
                        Eigen::Vector3d refractionDir = (normal * (dot)-viewDir) * ior - normal * std::sqrt(discriminate);

                        // Check occlusion for refraction ray
                        if (checkOccluded2(nullptr, refractionDir, hr.impactPos, minT, maxT, l, refractionCoef))
                            return true;
                    }
                    // only checking refraction right now
                    //  else
                    //  {
                    //      Eigen::Vector3d internalReflectionDir = (hr.rayDir + 2.0 * -hr.rayDir.dot(normal) * normal).normalized();

                    //     // Check occlusion for reflection ray
                    //     if (checkOccluded2(s, internalReflectionDir, hr.impactPos, minT, maxT, l))
                    //         return true;
                    // }
                    // double t = s->properties.T;
                    // double ior = s->properties.index_of_refraction;
                    // double rad = ((Sphere*)s)->radius;
                    // refractionCoef *= t;
                    // double efl =  ior * rad / (2 * (ior - 1));
                    // double bfl = efl - rad;
                    // refractionCoef *= (1 - )
                }
                else
                    return true;
        }

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            // If the child exists check if any solids within it occlude the surface
            if (children[i] != nullptr && children[i]->checkOccluded2(parentSolid, rayDir, e, minT, maxT, l, refractionCoef))
                return true;
        }

        return false;
    }

    // Accounts for refracting objects
    // If the only objects occluding this light are transparent, we will cast rays from the light at this position and see if we get close.
    bool checkOccluded3(Solid *parentSolid, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, bool &onlyTransparentOccluding, Solid *&closest)
    {
        // printf("in good check occluded\n");
        bool anything = false;
        // if (distFromCam == -1)
        // {
        //     distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
        // }

        //  first check if the ray intersects this octant
        // if (boundingSphere->radius > maxT / 2)
        //{
        if (boundingSphere->intersect(rayDir, e, minT, 10000000) >= 10000000) // WARNING!! This might cause issues since now we're checking against dist to light!!
            return false;
        //}
        // else if (boundingSphere->intersect(rayDir, e, minT, maxT * 2) >= maxT * 2) // WARNING!! This might cause issues since now we're checking against dist to light!!
        //    return false;
        HitRecord hr;
        hr.t = maxT;
        hr.nearest = nullptr;
        double t = maxT;
        double furthest = minT;
        // first check the solids that are directly in this octant
        for (Solid *s : solids)
        {
            t = maxT; //reset t
           
            if (parentSolid != s) // Had "&& s->type() != 0" here.
            {
                
                t = s->intersect(rayDir, e, minT, maxT, hr);
            }
            //
            if (t < maxT)
            {
                anything = true;
                if (t > furthest)
                {
                    furthest = t; // dist of object from surface
                    closest = s;  // transparent object closest to light
                }
                
                // if the solid is transparent we need to check if the refraction ray has LOS to the light.
                if (s->properties.T == 0)
                {                
                    onlyTransparentOccluding = false;
                    return true;
                }
                else if (s->TYPE() != "Sphere")
                { // type check here since I've only really tested spheres
                    // printf("got here\n");
                    onlyTransparentOccluding = false;
                    return true;
                }
            }
        }

        //  Now check if the ray intersects any of the child octants.
        for (int i = 0; i < 8; i++)
        {
            // If the child exists check if any solids within it occlude the surface
            if (children[i] != nullptr && children[i]->checkOccluded3(parentSolid, rayDir, e, minT, maxT, onlyTransparentOccluding, closest))
                return true;
        }

        return anything;
    }

    ~Octree()
    {
        // std::cout << "Destructor" << std::endl;

        for (int i = 0; i < 8; i++)
        {
            delete[] octantBounds[i];

            if (children[i] != nullptr)
                delete children[i];
        }

        delete boundingSphere;
    }

    // helper function for populating the Octrees' bounding vectors
    Eigen::Vector3d *makeBounds(double x, double y, double z, double size)
    { // makes the bounds based on origin x,y and the size
        Eigen::Vector3d *bounds = new Eigen::Vector3d[2];

        Eigen::Vector3d frontTopRight, backBottomLeft;

        // int[][] b = new int[3][2];
        frontTopRight << x, y, z;
        backBottomLeft << x + size, y + size, z + size;
        bounds[0] = frontTopRight;
        bounds[1] = backBottomLeft;

        return bounds;
    }

    // Populates the Octrees' bounding vectors
    void setupOctantBounds()
    {
        // std::cout << "Setting up octant bounds" << std::endl;

        double x = origin[0];
        double y = origin[1];
        double z = origin[2];

        // far octants first
        octantBounds[0] = makeBounds(x - size / 2, y - size / 2, z - size / 2, size / 2); // NW
        octantBounds[1] = makeBounds(x, y - size / 2, z - size / 2, size / 2);            // NE

        octantBounds[2] = makeBounds(x - size / 2, y, z - size / 2, size / 2); // SW
        octantBounds[3] = makeBounds(x, y, z - size / 2, size / 2);            // SE

        // close octants now (changing z)
        octantBounds[4] = makeBounds(x - size / 2, y - size / 2, z, size / 2); // NW
        octantBounds[5] = makeBounds(x, y - size / 2, z, size / 2);            // NE

        octantBounds[6] = makeBounds(x - size / 2, y, z, size / 2); // SW
        octantBounds[7] = makeBounds(x, y, z, size / 2);            // SE
        // std::cout << "Finished setting up octant bounds" << std::endl;
    }

    std::string toString(int spacer)
    {
        // std::cout << "in tostring1 " << std::endl;

        std::string data = "";
        std::string space = "";
        for (int i = 0; i < spacer * 4; i++)
        {
            space += " ";
        }
        if (solids.size() > 0)
            data += space + "Level " + std::to_string(spacer) + ", Size = " + std::to_string(size) + " with " + std::to_string(solids.size()) + " objects\n" + space + "\n";
        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                data += children[i]->toString(spacer + 1);
            }
        }
        // std::cout << "in tostring " << std::endl;

        return data;
    }
};

template <>
class Octree<Eigen::Vector3d *>
{
private:
    long long int getTime()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return ms.count();
    }

public:
    // The smallest sphere that can enclose the octant that this node defines.
    // Using this so I can check if a Ray (probably) intersects this node. A better way would be to actually check if a ray
    // intersects the cube this node defines, but its much easier to just do spheres since I already programmed that
    Sphere *boundingSphere;
    Octree *parent;
    std::vector<Octree *> children;
    int obsContained = 0; // how many solids are in this octant (or its children (recursive))

    double MIN_SIZE = .00000001; // the minimum side length an octant must be larger than to qualify for children
    int minObs = 10;             // the minimum number of objects in an octant to justify making a new octree node
    double size;                 // side length of this octant
    double distFromCam = -1;     // will be set properly by a function later
    std::vector<Eigen::Vector3d *> octantBounds;
    //  octants go in this order(if looking at a cube face on):
    /*
     * far away:
     * 	 north west, north east		|0,1|
     *   south west, south east		|2,3|
     * close side:
     * 								|4,5|
     * 								|6,7|
     */
    Eigen::Vector3d origin;

    std::unordered_set<Eigen::Vector3d *> octList[8];
    std::unordered_set<Eigen::Vector3d *> solids;
    std::unordered_set<Eigen::Vector3d *> allChildObs;
    // ArrayList <Freebody> [] octList=new ArrayList [8];//contains all the objects that fit within each respective octant.
    // ArrayList <Freebody> obs;
    bool leaf = true;
    Octree(Octree *_parent, Eigen::Vector3d *bounds, std::unordered_set<Eigen::Vector3d *> &objectsInOctant, int &counter)
        : Octree(_parent, bounds[0], bounds[1][0] - bounds[0][0], objectsInOctant, counter)
    {
    }

    Octree(Octree *_parent, Eigen::Vector3d roots, double size_, std::unordered_set<Eigen::Vector3d *> &objectsInOctant, int &counter)
    {
        allChildObs = objectsInOctant;
        counter++; // counts how many Octree nodes we've created
        // printf("Light oc size = %d\n", objectsInOctant.size());
        //  printf("\nSize = %f num obs = %d\n", size_, objectsInOctant.size());
        //  set some variables
        size = size_;
        // solids = objectsInOctant;
        parent = _parent;

        // origin is the center of the octant/octree
        origin[0] = roots[0] + size / 2;
        origin[1] = roots[1] + size / 2;
        origin[2] = roots[2] + size / 2;

        // calculate the radius of the bounding sphere
        double radiusOfBoundingSphere = (size * sqrt(3) / 2.0);

        // The boundingSphere for an octant is how we check for intersection of a ray with an octant.
        // While this overestimates the octant's volume and hitbox, that's okay.
        boundingSphere = new Sphere(ObjProps(), origin, radiusOfBoundingSphere);

        for (int i = 0; i < 8; i++)
        {
            octList[i] = std::unordered_set<Eigen::Vector3d *>();
        }
        // octantBounds = new Eigen::Vector3d *[8];
        octantBounds.reserve(8);
        setupOctantBounds();
        int numRemoved = 0;
        // sort the objects in this octant into the child octants.
        // Make sure this octant is able to have children.
        if (size / 2 > MIN_SIZE && objectsInOctant.size() > minObs)
            for (auto it = objectsInOctant.begin(); it != objectsInOctant.end();)
            {
                bool removed = false;
                for (int i = 0; i < 8; i++)
                {
                    // Get the bounding vectors for the current child
                    Eigen::Vector3d childCorner1 = octantBounds[i][0];
                    Eigen::Vector3d childCorner2 = octantBounds[i][1];
                    // printf("o size = %f, rad = %f\n", size, radiusOfBoundingSphere);
                    // std::cout << "o center : " << origin  << std::endl;

                    // Mini specialization for checking if a Solid is inside the octant vs if a vector is inside the octant

                    if (insideOctree(*it, childCorner1, childCorner2, origin))
                    {
                        numRemoved++;
                        octList[i].insert(*it);
                        break;
                    }
                }
                if (!removed)
                    it++;
            }

        if (numRemoved == objectsInOctant.size())
            leaf = false;
        // preallocate space
        children.reserve(8);

        // creating the children octrees once objects have been determined to lie within them
        for (int i = 0; i < 8; i++)
        {
            // printf("Octant %d has %d obs in it\n", i, octList[i].size());
            // only create a child if it will have any solids in it
            if (octList[i].size() > 0)
            {
                // std::cout << "Created child node. obs in child = " << octList[i].size() << std::endl;
                children[i] = new Octree<Eigen::Vector3d *>(this, octantBounds[i][0], octantBounds[i][1][0] - octantBounds[i][0][0], octList[i], counter);
            }
            else
            {
                children[i] = nullptr;
            }
        }

        // The code commented out below code is for validation and testing optimizations. You can ignore it.

        // long long int t2 = getTime();
        // counter += (t2 - t1);
        // make sure we didn't mistakenly discard any solids or duplicate something
        // std::unordered_set<T> total;
        // validateObjectCount(total);
        // obsContained = total.size();
        // if (total.size() != objectsInOctant.size())
        // {
        //     printf("something went wrong %d\n", (total.size() - objectsInOctant.size()));
        // }

        // if (total.size() == 0)
        // {
        //     printf("This octant is useless\n");
        // }

        // std::unordered_set<T> childrenObs;
        // getObsInChildren(childrenObs);
        // for (T s : childrenObs)
        //     allChildObs.insert(s);

        // std::cout << "Finished constructor. obs in octant = " << solids.size() << std::endl;
    }

    bool insideOctree(Eigen::Vector3d *pos, Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d origin)
    {
        bool vertexInside = true;
        // loop over every dimension
        for (int dim = 0; dim < 3; dim++)
        {
            vertexInside = vertexInside && (corner1[dim] <= (*pos)[dim] && (*pos)[dim] <= corner2[dim]);
        }

        // printf("check inside, size = %f\n", size);
        return vertexInside;
    }

    // template <typename Q>
    // double getIntensity(double maxDist, Eigen::Vector3d pos, int& c) { printf("default 2\n"); return 0; }

    // Function to test runtime optimizations.
    // When called at the root of the Octree, it will return the number of octrees that
    // theoritically could be replace by a child node, without disturbing any data.
    // Only true if this octant if
    //   a) Doesn't have any solids in it
    //   AND
    //   b) Only has 1 child octant with solids in it
    int countReplaceable()
    {
        int count = 0;
        int numChildren = 0;
        bool replaceable = true;
        if (solids.size() > 0)
            replaceable = false;

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                numChildren++;
                count += children[i]->countReplaceable();
            }
        }

        if (numChildren < 2 && solids.size() == 0)
            replaceable = true;
        if (replaceable)
            return count + 1;
        else
            return count;
    }

    // makes sure that the number of objects in this octant plus the number of objects in all
    // children (recursive) equals the number of objects that was originally passed to this Octree node.
    void validateObjectCount(std::unordered_set<Eigen::Vector3d *> &tot)
    {
        for (Eigen::Vector3d *s : solids)
            tot.insert(s);

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                children[i]->validateObjectCount(tot);
            }
        }
    }

    // Populates tot with all the children in this octant or its children (recursive)
    void getObsInChildren(std::unordered_set<Eigen::Vector3d *> &tot)
    {
        for (std::unordered_set<Eigen::Vector3d *> childrenObs : octList)
            for (Eigen::Vector3d *s : childrenObs)
                tot.insert(s);

        for (int i = 0; i < 8; i++)
        {
            if (children[i] != nullptr)
            {
                children[i]->getObsInChildren(tot);
            }
        }
    }

    ~Octree()
    {
        // std::cout << "Destructor" << std::endl;

        for (int i = 0; i < 8; i++)
        {
            delete[] octantBounds[i];

            if (children[i] != nullptr)
                delete children[i];
        }

        delete boundingSphere;
    }

    // helper function for populating the Octrees' bounding vectors
    Eigen::Vector3d *makeBounds(double x, double y, double z, double size)
    { // makes the bounds based on origin x,y and the size
        Eigen::Vector3d *bounds = new Eigen::Vector3d[2];

        Eigen::Vector3d frontTopRight, backBottomLeft;

        // int[][] b = new int[3][2];
        frontTopRight << x, y, z;
        backBottomLeft << x + size, y + size, z + size;
        bounds[0] = frontTopRight;
        bounds[1] = backBottomLeft;

        return bounds;
    }

    // Populates the Octrees' bounding vectors
    void setupOctantBounds()
    {
        // std::cout << "Setting up octant bounds" << std::endl;

        double x = origin[0];
        double y = origin[1];
        double z = origin[2];

        // far octants first
        octantBounds[0] = makeBounds(x - size / 2, y - size / 2, z - size / 2, size / 2); // NW
        octantBounds[1] = makeBounds(x, y - size / 2, z - size / 2, size / 2);            // NE

        octantBounds[2] = makeBounds(x - size / 2, y, z - size / 2, size / 2); // SW
        octantBounds[3] = makeBounds(x, y, z - size / 2, size / 2);            // SE

        // close octants now (changing z)
        octantBounds[4] = makeBounds(x - size / 2, y - size / 2, z, size / 2); // NW
        octantBounds[5] = makeBounds(x, y - size / 2, z, size / 2);            // NE

        octantBounds[6] = makeBounds(x - size / 2, y, z, size / 2); // SW
        octantBounds[7] = makeBounds(x, y, z, size / 2);            // SE
        // std::cout << "Finished setting up octant bounds" << std::endl;
    }
};

// template <>
// bool Octree<Eigen::Vector3d *>::insideOctree<Eigen::Vector3d *>(Eigen::Vector3d *pos, Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d origin)
// {
//     bool vertexInside = true;
//     // loop over every dimension
//     for (int dim = 0; dim < 3; dim++)
//     {
//         vertexInside = vertexInside && (corner1[dim] <= (*pos)[dim] && (*pos)[dim] <= corner2[dim]);
//     }
//     // printf("check inside, size = %f\n", size);
//     return vertexInside;
// }

// template <>
// double Octree<Eigen::Vector3d *>::getIntensity<Eigen::Vector3d*>(double maxDist, Eigen::Vector3d pos, int& c)
// {
//     printf("get inten\n");
//     // calculate intensity based on all pnts in this octant (and its children (recursive))
//     if (size < maxDist)
//     {
//         //right now just calculates total without coefs
//         double total = 0;
//         int i = 0;

//         for (Eigen::Vector3d* v : allChildObs)
//         {
//             double dist = ((*v) - pos).norm();
//             if (dist < maxDist)
//                 c++;
//             total += (1.0 / std::pow(1 + dist, 2));
//             //}
//             // total += coefs[i] * (1.0 / std::pow((1 + dist), 2));
//             // total += coefs[i] * (1.0 / std::pow((dist + pixWidth / 2), 2));
//             i++;
//         }
//         return total;
//     }
//     else
//     {
//         //traverse down the Octree
//         for (int i = 0; i < 8; i++)
//         {
//             // Get the bounding vectors for the current child
//             Eigen::Vector3d childCorner1 = octantBounds[i][0];
//             Eigen::Vector3d childCorner2 = octantBounds[i][1];
//             // printf("o size = %f, rad = %f\n", size, radiusOfBoundingSphere);
//             // std::cout << "o center : " << origin  << std::endl;

//             // Mini specialization for checking if a Solid is inside the octant vs if a vector is inside the octant

//             if (insideOctree(&pos, childCorner1, childCorner2, origin)){
//                 return children[i]->getIntensity(maxDist, pos, c);
//             }
//         }
//     }

//     return 0;
// }

#endif
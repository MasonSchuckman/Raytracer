// File to store basic intersection code. Allows octree to inherit from it and overload the functions.
#include <Eigen/Dense>
#include "Solid.h"
#include <vector>

#ifndef Intersecter_H
#define Intersecter_H

class Intersecter
{

public:



    Intersecter() {}

    virtual void intersect(std::vector<Solid*> & solids, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, HitRecord &hr){
        
        // loop over every solid in the scene and check for intersection
        // hit info gets saved in hit record
        for (Solid *s : solids)
        {
            double intersectionS = s->intersect(rayDir, e, minT, maxT, hr);            
        }
    }

    virtual bool checkOccluded(std::vector<Solid*> & solids, Solid *parentSolid, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT){
        // check if any objects are in between the impact point and the light
        for (Solid *s : solids)
        {
            if (parentSolid != s && s->intersect(rayDir, e, minT, maxT) != maxT)
                return true;
        }

        return false;
    }

    virtual pixelInfo intersect(std::vector<Solid*> & solids, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, double _nearest)
    {

        double intersectionT = maxT;

        Solid *closest = nullptr;

        // loop over every solid in the scene and check for intersection
        // save info about the closest intersection.
        for (Solid *s : solids)
        {
            double intersectionS = s->intersect(rayDir, e, minT, intersectionT);
            // if(intersectionS > 0 && intersectionS != maxT)
            //     std::cout << "valid intersection\n";
            if (intersectionT > intersectionS)
            {
                // if(intersectionT = maxT)
                //     std::cout << "New closest!\n";
                intersectionT = intersectionS;
                closest = s;
            }
        }

        return pixelInfo{closest, intersectionT};
    }
};

#endif

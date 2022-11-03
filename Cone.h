#include "Solid.h"

#ifndef Cone_H
#define Cone_H

//class for BOTH cone and cylinder
class Cone : public Solid{
public: 
    Eigen::Vector3d bot;
    Eigen::Vector3d top;
    double botRad, topRad;


    //used if we add verticies as we read them in from the nff file
    Cone(ObjProps props, Eigen::Vector3d bottom, Eigen::Vector3d top, double bottomRadius, double topRadius)
        : Solid(props), bot(bottom), top(top), botRad(bottomRadius), topRad(topRadius) {       
    }

    

    // //TODO (maybe): change how this works. Right now it looks bad.
    // std::string toString(){
    //     return "Position:\n" + Entity::toString() + "\n Color: " +  to_string(r) + " " + to_string(g) + " " + to_string(b) + "\n";
    // }
    
};

#endif
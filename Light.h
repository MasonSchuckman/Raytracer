#include "Entity.h"

#ifndef Light_H
#define Light_H

class Light : public Entity{
public: 
    double r,g,b;
    Eigen::Vector3d color;
    double intensity = 1; //1/sqrt(num lights)

    //constructoring using initializer list for parent class and this class
    Light(double r, double g, double b, double x, double y, double z) : Entity(x,y,z)
    , r(r), g(g), b(b){
        color << r,g,b;
    };

    Light(Eigen::Vector3d rgb, Eigen::Vector3d pos) : Entity(pos), color(rgb)
    , r(rgb[0]), g(rgb[1]), b(rgb[2]){
    };


    // //TODO (maybe): change how this works. Right now it looks bad.
    // std::string toString(){
    //     return "Position:\n" + Entity::toString() + "\n Color: " +  std::to_string(r) + " " + std::to_string(g) + " " + std::to_string(b) + "\n";
    // }
};

#endif
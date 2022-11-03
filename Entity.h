#include <Eigen/Dense>
#include <iostream>

#ifndef Entity_H
#define Entity_H

class Entity{
public: 
    Eigen::Vector3d pos;

    Entity(double x, double y, double z){
        pos<<x,y,z;
    }

    Entity(Eigen::Vector3d pos) : pos(pos){}

    std::string toString(){
        std::stringstream s;
        s << pos;
        return s.str();      
    }

};

#endif
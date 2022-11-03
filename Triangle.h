#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "Solid.h"

#ifndef Triangle_H
#define Triangle_H

class Triangle : public Solid
{
public:
    Eigen::Vector3d normal;
    std::vector<Eigen::Vector3d> normals; // used if this triangle was created from a polygonal patch

    bool first = true;
    Triangle(ObjProps props) : Solid(props)
    {
    }

    // used if we create and populate the verticies vector while reading the nff file THEN create the polygon.
    Triangle(ObjProps props, std::vector<Eigen::Vector3d> verts) : Solid(props, verts)
    {
    }

    void addVertex(Eigen::Vector3d vertex)
    {
        verts.push_back(vertex);
    }

    // used if this triangle was created from a PP
    void addVertex(Eigen::Vector3d vertex, Eigen::Vector3d normal)
    {
        verts.push_back(vertex);
        normals.push_back(normal);
    }

    Eigen::Vector3d ab, ac, ae;

    // precompute and pre-allocate memory for efficiency
    void finalize()
    {
        ab = verts[0] - verts[1];
        ac = verts[0] - verts[2];
        normal = ab.cross(ac).normalized();
        // normal = -normal;
    }

    // creates 4 sub triangles
    Triangle **subdivide()
    {
        Triangle **subsections = new Triangle *[4];
        Eigen::Vector3d a = verts[0];
        Eigen::Vector3d b = verts[1];
        Eigen::Vector3d c = verts[2];

        Eigen::Vector3d one = (b + a) / 2;
        Eigen::Vector3d two = (c + b) / 2;
        Eigen::Vector3d three = (a + c) / 2;

        subsections[0] = new Triangle(properties, std::vector<Eigen::Vector3d>({a, one, three}));
        subsections[1] = new Triangle(properties, std::vector<Eigen::Vector3d>({one, b, two}));
        subsections[2] = new Triangle(properties, std::vector<Eigen::Vector3d>({three, two, c}));
        subsections[3] = new Triangle(properties, std::vector<Eigen::Vector3d>({one, two, three}));

        for (int i = 0; i < 4; i++)
            subsections[i]->finalize();

        return subsections;
    }

    double calcDistanceToCam(Eigen::Vector3d &e)
    {
        // if (distToCam == -1)
        // {
        //     distToCam = 100000000000;
        //     for (int i = 0; i < 3; i++)
        //     {
        //         distToCam = std::min(distToCam, (verts[i] - e).norm());
        //     }
        // }
        // return distToCam;
        return -1;
    }


    
    

    // proj2
    //Switched to using the prof's det() function instead of Eigen since it's much faster.
    double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, HitRecord &hr)
    {
        
        // Since we're dealing with relfections and such, we can no longer calculate this just once per frame.
        // Later on I might calculate this once per frame and use it only for 1st intersections, but right now I'm leaving it out.
        // That's why first=false is commented out.
        // if (first)
        Eigen::Vector3d ae = verts[0] - e;
        // first = false;

        //Eigen::Matrix3d A(3, 3);
        //A << ab, ac, rayDir;

        double detA = det(ab,ac,rayDir); //A.determinant();

        
        //Eigen::Matrix3d ALPHA, BETA, T;

        double alpha, beta, t;

        // following the book but my some of variable names came from lecture :/
        // check alpha
        //ALPHA = Eigen::Matrix3d(3, 3);
        //ALPHA << ae, ac, rayDir;
        //alpha = ALPHA.determinant() / detA;
        alpha = det(ae, ac, rayDir) / detA;
        if (alpha < 0 || alpha > 1)
            return maxT;

        // std::cout << "check beta\n";
        // check beta
        //BETA = Eigen::Matrix3d(3, 3);
        //BETA << ab, ae, rayDir;
        //beta = BETA.determinant() / detA;
        beta = det(ab, ae, rayDir) / detA;
        if (beta < 0 || beta > 1 - alpha)
            return maxT;

        // std::cout << "check T\n";
        // check t
        //T = Eigen::Matrix3d(3, 3);
        //T << ab, ac, ae;
        //t = T.determinant() / detA;
        t = det(ab, ac, ae) / detA;
        if (t < minT || t > maxT)
            return maxT;
        //if(maxT < 50)
        //printf("tri t = %f\n", t);
        // check if this triangle is from a polygonal patch, if it is, we need to calculate the normal based on interpolating the vertices normal vectors
        // printf("norms\n");
        Eigen::Vector3d normal_ = normal;

        // I think this works? not 100% sure
         if(!normals.empty() && phong){
             //normal_ =  alpha * normals[1] + beta * normals[2] + t * normals[0];
             normal_ =  alpha * normals[1] + beta * normals[2] + (1 - (alpha + beta)) * normals[0];
             // for(int i = 0; i < 3; i++){
             //     printf("normals : %f %f %f\n", normals[i][0], normals[i][1], normals[i][2]);

            // }
            //normal_ = normal_.eval();
        }

        updateHitRecord(t, rayDir, e, normal_, hr);


        return t;
    }

    virtual Eigen::Vector3d getNormal(Eigen::Vector3d & rayDir){
        return normal;
    }

    // mostly from notes and around page 78 of textbook
    double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT)
    {

        //  Eigen::Vector3d a,b,c;
        //  a = verts[0];
        //  b = verts[1];
        //  c = verts[2];

        // I'm opting for cleaner code over efficiency to start with.
        // Might change how I'm doing math here later for better performance.

        // if (first)
        Eigen::Vector3d ae = verts[0] - e;
        first = false;
        //Eigen::Matrix3d A(3, 3);
        //A << ab, ac, rayDir;
        // std::cout << "1 - 0: " << verts[1] - verts[0] << std::endl;
        // std::cout << "2 - 0: " << verts[2] - verts[0] << std::endl;
        // std::cout << "-rayd: " << (-rayDir) << std::endl;

        // std::cout << "Matrix A: " << A << std::endl;
        // int ran = rand() % 1000;

        double detA = det(ab,ac,rayDir); //A.determinant();

        // Eigen::Matrix3d inv = A.inverse();
        // //std::cout << "Inverse: " << inv << std::endl;

        // Eigen::Vector3d solution = inv * (verts[0] - e);

        // std::cout << "Solution: " << solution << std::endl;
        // t = solution[0];
        // alpha = solution[1];
        // beta = solution[2];
        //Eigen::Matrix3d ALPHA, BETA, T;

        double alpha, beta, t;

        // following the book but my some of variable names came from lecture :/
        // check alpha
        //ALPHA = Eigen::Matrix3d(3, 3);
        //ALPHA << ae, ac, rayDir;
        //alpha = ALPHA.determinant() / detA;
        alpha = det(ae, ac, rayDir) / detA;
        if (alpha < 0 || alpha > 1)
            return maxT;

        // std::cout << "check beta\n";
        // check beta
        //BETA = Eigen::Matrix3d(3, 3);
        //BETA << ab, ae, rayDir;
        //beta = BETA.determinant() / detA;
        beta = det(ab, ae, rayDir) / detA;
        if (beta < 0 || beta > 1 - alpha)
            return maxT;

        // std::cout << "check T\n";
        // check t
        //T = Eigen::Matrix3d(3, 3);
        //T << ab, ac, ae;
        //t = T.determinant() / detA;
        t = det(ab, ac, ae) / detA;
        if (t < minT || t > maxT)
            return maxT;

        // std::cout << "good hit!\n";
        return t;
    }

    virtual Eigen::Vector4d raytrace(Eigen::Vector3d rayDir, Eigen::Vector3d e, double minT, double maxT, int reflectionsRemaining, std::vector<Solid *> &solids, Solid *parent)
    {
        // std::cout << "started inter in triangle!\n";
        //  Eigen::Vector3d a,b,c;
        //  a = verts[0];
        //  b = verts[1];
        //  c = verts[2];

        // Might change how I'm doing math here later for better performance.
        Eigen::Vector3d color_ = properties.color;

        Eigen::Vector4d color;
        // color << color_, Eigen::VectorXd{0};
        color << color_[0], color_[1], color_[2], 0;

        if (first)
            ae = verts[0] - e;
        first = false;
        Eigen::Matrix3d A(3, 3);
        A << ab, ac, rayDir;
        // std::cout << "1 - 0: " << verts[1] - verts[0] << std::endl;
        // std::cout << "2 - 0: " << verts[2] - verts[0] << std::endl;
        // std::cout << "-rayd: " << (-rayDir) << std::endl;

        // std::cout << "Matrix A: " << A << std::endl;
        // int ran = rand() % 1000;

        double detA = A.determinant();

        // Eigen::Matrix3d inv = A.inverse();
        // //std::cout << "Inverse: " << inv << std::endl;

        // Eigen::Vector3d solution = inv * (verts[0] - e);

        // std::cout << "Solution: " << solution << std::endl;
        // t = solution[0];
        // alpha = solution[1];
        // beta = solution[2];
        Eigen::Matrix3d ALPHA, BETA, T;
        ALPHA = Eigen::Matrix3d(3, 3);

        double alpha, beta, t;

        // following the book but my some of variable names came from lecture :/
        // check alpha

        ALPHA << ae, ac, rayDir;
        alpha = ALPHA.determinant() / detA;

        if (alpha < 0 || alpha > 1)
            return Eigen::Vector4d{0, 0, 0, maxT};

        // std::cout << "check beta\n";
        // check beta
        BETA = Eigen::Matrix3d(3, 3);
        BETA << ab, ae, rayDir;
        beta = BETA.determinant() / detA;
        if (beta < 0 || beta > 1 - alpha)
            return Eigen::Vector4d{0, 0, 0, maxT};

        // std::cout << "check T\n";
        // check t
        T = Eigen::Matrix3d(3, 3);
        T << ab, ac, ae;
        t = T.determinant() / detA;
        if (t < minT || t > maxT)
            return Eigen::Vector4d{0, 0, 0, maxT};

        // std::cout << "good hit!\n";
        color[3] = t;

        if (reflectionsRemaining == 0)
        {
            return color;
        }
        else
        {

            Eigen::Vector3d reflectionPoint = rayDir * t + e;
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
                if (s != this && s != parent)
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
    }

    // Checks if this solid is at all inside a node of an octree.
    bool insideOctree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d center)
    {
        // loop over every vertex and check if its within the cube. if any are in the octant then the triangle intersects the octant.
        bool vertexInside = true;

        for (int i = 0; i < 3; i++)
        {
            for (int dim = 0; dim < 3; dim++)
            {
                vertexInside = vertexInside && (corner1[dim] <= verts[i][dim] && verts[i][dim] <= corner2[dim]);
            }
        }
        // if (vertexInside)
        //     return true;
        return vertexInside;
    }

    virtual string TYPE()
    {
        return "Triangle";
    }

    int type(){
        return 0;
    }
    
    //functions from prof's code below

    // return the determinant of the matrix with columns a, b, c.
    double det(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c)
    {
        return a[0] * (b[1] * c[2] - c[1] * b[2]) +
               b[0] * (c[1] * a[2] - a[1] * c[2]) +
               c[0] * (a[1] * b[2] - b[1] * a[2]);
    }

    inline double sqr(double x) { return x * x; }
    inline Eigen::Vector3d cross(const Eigen::Vector3d &x, const Eigen::Vector3d &y) { return x.cross(y); }


};

#endif
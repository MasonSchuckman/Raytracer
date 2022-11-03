#include "Solid.h"
#include "Triangle.h"

#ifndef Polygon_H
#define Polygon_H

class Polygon : public Solid
{
public:
    // vector containing the triangles that compose this polygon
    std::vector<Triangle *> triangles;

    Eigen::Vector3d normal_;

    int numVerticies;

    // used if we add verticies as we read them in from the nff file
    Polygon(ObjProps props, int numVerticies) : Solid(props), numVerticies(numVerticies)
    {
    }

    // used if we create and populate the verticies vector while reading the nff file THEN create the polygon.
    Polygon(ObjProps props, int numVerticies, std::vector<Eigen::Vector3d> verts) : Solid(props, verts), numVerticies(numVerticies)
    {
    }

    ~Polygon()
    {
        for (int i = 0; i < triangles.size(); i++)
            delete triangles[i];
    }

    void addVertex(Eigen::Vector3d vertex)
    {
        verts.push_back(vertex);
    }

    // used to make smaller triangles. Useful for octree segmentation so we don't "miss" a triangle.
    void subdivide(Triangle *tri, int levelsLeft)
    {
        if (levelsLeft == 0)
        {
            triangles.push_back(tri);
        }
        else
        {
            Triangle **sub = tri->subdivide();
            for (int i = 0; i < 4; i++)
            {
                subdivide(sub[i], levelsLeft - 1);
            }
            delete tri;
        }
    }

    // creates and adds a triangle to the vertex list for this polygon
    virtual void addTriangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c)
    {
        Triangle *tri = new Triangle(properties);
        tri->addVertex(a);
        tri->addVertex(b);
        tri->addVertex(c);

        tri->finalize();

        // this subdivision thing was written because I thought it might be able to improve the program's
        // performance when using Octrees, but turns out it just makes things slower.
        // Possibly because I didn't implement something else efficiently
        subdivide(tri, 0);

        // triangles.push_back(tri);
    }

    // triangulates the polygon.
    // TODO: implement ear clipping algorithm
    void fanTriangles()
    {
        for (int i = 0; i < numVerticies - 2; i++)
        {
            addTriangle(verts[0], verts[i + 1], verts[i + 2]);

            // Triangle* tri = new Triangle(properties);
            // tri->addVertex(verts[0]);
            // tri->addVertex(verts[i + 1]);
            // tri->addVertex(verts[i + 2]);

            // tri->finalize();
            // triangles.push_back(tri);
        }
        // std::cout << "Polygon with num triangles = " << triangles.size() << std::endl;
    }

    // from  https://math.stackexchange.com/questions/51326/determining-if-an-arbitrary-point-lies-inside-a-triangle-defined-by-three-points (main answer)
    //  checks if q is inside (a,b,c).
    //  Does this by checking if the q is on the right of ab,bc and ca. If its always on the right hand side,
    //  then its within the bounds of a,b,c.
    bool checkPointInTriangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d q)
    {

        double s1 = ((a - b).cross(a - q))[2];
        double s2 = ((b - c).cross(b - q))[2];
        double s3 = ((c - a).cross(c - q))[2];
        return (s1 > 0 && s2 > 0 && s3 > 0) || (s1 < 0 && s2 < 0 && s3 < 0);
        
    }

    void naiveEarClipping()
    {
        normal_ = ((verts[0] - verts[1]).cross(verts[2] - verts[1])).normalized();
        // make a copy of our verticies (This is slow but that's why its called "naive" ear clipping)
        std::vector<Eigen::Vector3d> v = verts;

        // for(int i = 0; i < v.size(); i++){
        //     printf("(%f,%f,%f),", v[i][0], v[i][1], v[i][2]);
        // }
        // printf("\n");

        double deg = 180.0 / 3.14159;
        for (int i = 0; i < v.size() - 3; i++)
        {
            int vSize = v.size();
            // printf("i = %d, vsize = %d\n", i, vSize);

            // get the 3 verticies we're considering
            Eigen::Vector3d a = v[(i - 1 + vSize) % vSize]; // add vSize here to deal with negative
            Eigen::Vector3d b = v[i];
            Eigen::Vector3d c = v[(i + 1) % vSize];

            // check if the vertices are convex
            Eigen::Vector3d ba = b - a;
            Eigen::Vector3d bc = b - c;

            // doesn't seem to work
            // double angle = acos(bc.dot(ba) / sqrt(ba.squaredNorm() * bc.squaredNorm())) * deg;

            // the math behind these 3 lines of code was taken from https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors (accepted answer)
            Eigen::Matrix3d m(3, 3);
            m << ba, bc, normal_;
            double angle = atan2(m.determinant(), ba.dot(bc));

            angle *= deg;

            if (angle > 180 || angle < 0)
                continue;

            // bool sameDir = true;
            // for (int dim = 0; dim < 3; dim++)
            // {
            //     sameDir = sameDir && ba.cross(bc)[dim] > 0 && normal_[dim] > 0 ||
            //               ba.cross(bc)[dim] < 0 && normal_[dim] < 0;
            // }
            // if (!sameDir)
            // {

            //     printf("FAIL:\n");
            //     printf("(%f,%f,%f),", ba[0], ba[1], ba[2]);
            //     printf("(%f,%f,%f),", bc[0], bc[1], bc[2]);
            //     printf(" and size = %d, angle = %f\n", vSize, angle);
            //     continue;
            // }
            // else
            // {
            //     printf("PASS:\n");
            //     printf("(%f,%f,%f),", ba[0], ba[1], ba[2]);
            //     printf("(%f,%f,%f),", bc[0], bc[1], bc[2]);
            //     printf(" and size = %d, angle = %f\n", vSize, angle);
            // }
            // now we check if any other verticies are within (a,b,c).
            // loop through all the other verticies
            bool ear = true;
            for (int test = 0; test < vSize; test++)
            {
                // dont test a vertex that is apart of the triangle
                if (test != (i - 1 + vSize) % vSize && test != i && test != (i + 1) % vSize)
                    if (checkPointInTriangle(a, b, c, v[test]))
                    {
                        // test = 10000000;
                        //  if a point is found within the triangle we can exit early since this isn't an ear.
                        ear = false;
                        break;
                    }
            }
            // go to next iteration if not an ear
            if (!ear)
            {
                // printf("Didnt find an ear!\n");

                continue;
            }
            else
            {
                // printf("Found an ear!\n");
                addTriangle(a, b, c);

                // remove the current vertex from the list
                v.erase(v.begin() + i);

                // go back to the beginning of the vertex list
                i = -1;
            }
        }

        // once we have 3 verticies left we can simply make a triangle out of them.
        addTriangle(v[0], v[1], v[2]);
    }

    double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, HitRecord & hr)
    {
        double nearest = maxT;
        for (int tri = 0; tri < triangles.size(); tri++)
        {
            nearest = std::min(nearest, triangles[tri]->intersect(rayDir, e, minT, maxT, hr));
            if (nearest != maxT)
                return nearest;
        }
        // if(nearest != maxT)
        //     std::cout << nearest << std::endl;
        return nearest;
    }

    // Checks for an intersection with any of the triangles in this polygon. Since polygons are 2d, only
    // one intersection is possible, so we can return early once we get a good intersection.
    double intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT)
    {
        double nearest = maxT;
        for (int tri = 0; tri < triangles.size(); tri++)
        {
            nearest = std::min(nearest, triangles[tri]->intersect(rayDir, e, minT, maxT));
            if (nearest != maxT)
                return nearest;
        }
        // if(nearest != maxT)
        //     std::cout << nearest << std::endl;
        return nearest;
    }


    //IGNORE FOR PROJ 1

    Eigen::Vector4d raytrace(Eigen::Vector3d rayDir, Eigen::Vector3d e, double minT, double maxT,
                             int reflectionsRemaining, std::vector<Solid *> &solids, Solid *parent)
    {
        double intersectionT = maxT;
        Triangle *closest = nullptr;
        Eigen::Vector4d color{0, 0, 0, maxT};
        for (Triangle *t : triangles)
        {
            Eigen::Vector4d possibleColor = t->raytrace(rayDir, e, minT, intersectionT, reflectionsRemaining, solids, this);

            if (intersectionT > possibleColor[3] && possibleColor[3] > 0)
            {
                intersectionT = possibleColor[3];
                closest = t;
                color = possibleColor;
            }
        }

        return color;
    }

    
    // I could just call the Triangle version of this like I do for intersect(), but I think its more efficient to just
    // loop through all the verticies here.
    bool insideOctree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d center)
    {
        bool vertexInside = true;
        // loop over every vertex and check if its within the cube. if any are in the octant then the triangle intersects the octant.
        for (int i = 0; i < verts.size(); i++)
        {
            for (int dim = 0; dim < 3; dim++)
            {
                vertexInside = vertexInside && (corner1[dim] <= verts[i][dim] && verts[i][dim] <= corner2[dim]);
            }
        }
        return vertexInside;
    }
};

#endif
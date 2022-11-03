#include "Polygon.h"
#include "Triangle.h"

#ifndef PolygonalPatch_H
#define PolygonalPatch_H

class PolygonalPatch : public Polygon
{
public:
    // vector containing the triangles that compose this polygon
    std::vector<Eigen::Vector3d> normals;


    // used if we add verticies as we read them in from the nff file
    PolygonalPatch(ObjProps props, int numVerticies) : Polygon(props, numVerticies)
    {
        //normals = std::vector<Eigen::Vector3d>(numVerticies);
    }

    // used if we create and populate the verticies vector while reading the nff file THEN create the polygon.
    PolygonalPatch(ObjProps props, int numVerticies, std::vector<Eigen::Vector3d> verts) : Polygon(props, numVerticies, verts)
    {
        //normals = std::vector<Eigen::Vector3d>(numVerticies);
    }

    ~PolygonalPatch()
    {
        for (int i = 0; i < triangles.size(); i++)
            delete triangles[i];
    }

    void addVertex(Eigen::Vector3d vertex, Eigen::Vector3d normal)
    {
        verts.push_back(vertex);
        normals.push_back(normal.normalized());
    }

    // creates and adds a triangle to the vertex list for this polygon
    // difference for PP versus Polygon is that PP needs to pass the normals of each vertex to the triangle it creates.
    virtual void addTriangle(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c, Eigen::Vector3d n1, Eigen::Vector3d n2, Eigen::Vector3d n3)
    {
        Triangle *tri = new Triangle(properties);
        tri->addVertex(a, n1);
        tri->addVertex(b, n2);
        tri->addVertex(c, n3);

        tri->finalize();

        // this subdivision thing was written because I thought it might be able to improve the program's
        // performance when using Octrees, but turns out it just makes things slower.
        // Possibly because I didn't implement something else efficiently
        subdivide(tri, 0);

        // triangles.push_back(tri);
    }

    // triangulates the polygon.
    void fanTriangles()
    {
        for (int i = 0; i < numVerticies - 2; i++)
        {
            addTriangle(verts[0], verts[i + 1], verts[i + 2], normals[0], normals[i + 1], normals[i + 2]);
        }
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
        std::vector<Eigen::Vector3d> norms = normals;

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
            int indA = (i - 1 + vSize) % vSize;
            int indB = i;
            int indC = (i + 1) % vSize;
            Eigen::Vector3d a = v[indA]; // add vSize here to deal with negative
            Eigen::Vector3d b = v[indB];
            Eigen::Vector3d c = v[indC];
            
            // check if the vertices are convex
            Eigen::Vector3d ba = b - a;
            Eigen::Vector3d bc = b - c;

            // the math behind these 3 lines of code was taken from https://stackoverflow.com/questions/14066933/direct-way-of-computing-clockwise-angle-between-2-vectors (accepted answer)
            Eigen::Matrix3d m(3, 3);
            m << ba, bc, normal_;
            double angle = atan2(m.determinant(), ba.dot(bc));

            angle *= deg;

            if (angle > 180 || angle < 0)
                continue;


            //now that we know the triangle formed by a,b,c is convex, we can check if it's a valid ear
            bool ear = true;
            for (int test = 0; test < vSize; test++)
            {
                // dont test a vertex that is apart of the triangle
                if (test != indA && test != indB && test != indC)
                    if (checkPointInTriangle(a, b, c, v[test]))
                    {
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
                addTriangle(a, b, c, norms[indA], norms[indB], norms[indC]);

                // remove the current vertex from the list
                v.erase(v.begin() + i);
                norms.erase(norms.begin() + i);

                // go back to the beginning of the vertex list
                i = -1;
            }
        }

        // once we have 3 verticies left we can simply make a triangle out of them.
        if(v.size() < 3 || norms.size() < 3){
            printf("Error in polygonal patch\n");
        }
        addTriangle(v[0], v[1], v[2], norms[0], norms[1], norms[2]);
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
};

#endif
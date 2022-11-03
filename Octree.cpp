// #include "Octree.h"
// #include "Sphere.h"
// #include "Solid.h"

// Octree::Octree(Octree *_parent, Eigen::Vector3d *bounds, const std::unordered_set<Solid *> objectsInOctant, int &counter)
//     : Octree(_parent, bounds[0], bounds[1][0] - bounds[0][0], objectsInOctant, counter)
// {
// }

// Octree::Octree(Octree *_parent, Eigen::Vector3d roots, double size_, const std::unordered_set<Solid *> objectsInOctant, int &counter)
// {
//     counter++; // counts how many Octree nodes we've created

//     // printf("\nSize = %f num obs = %d\n", size_, objectsInOctant.size());
//     // set some variables
//     size = size_;
//     solids = objectsInOctant;
//     parent = _parent;

//     // origin is the center of the octant/octree
//     origin[0] = roots[0] + size / 2;
//     origin[1] = roots[1] + size / 2;
//     origin[2] = roots[2] + size / 2;

//     // calculate the radius of the bounding sphere
//     double radiusOfBoundingSphere = (size * sqrt(3) / 2.0);

//     // The boundingSphere for an octant is how we check for intersection of a ray with an octant.
//     // While this overestimates the octant's volume and hitbox, that's okay.
//     boundingSphere = new Sphere(ObjProps(), origin, radiusOfBoundingSphere);

//     for (int i = 0; i < 8; i++)
//     {
//         octList[i] = std::unordered_set<Solid *>();
//     }
//     // octantBounds = new Eigen::Vector3d *[8];
//     octantBounds.reserve(8);
//     setupOctantBounds();

//     std::unordered_set<Solid *> rem(solids.size());

//     // sort the objects in this octant into the child octants.
//     // Make sure this octant is able to have children.
//     if (size / 2 > MIN_SIZE && solids.size() > minObs)
//         for (Solid *s : solids)
//         {
//             for (int i = 0; i < 8; i++)
//             {
//                 // Get the bounding vectors for the current child
//                 Eigen::Vector3d childCorner1 = octantBounds[i][0];
//                 Eigen::Vector3d childCorner2 = octantBounds[i][1];
//                 // printf("o size = %f, rad = %f\n", size, radiusOfBoundingSphere);
//                 // std::cout << "o center : " << origin  << std::endl;

//                 if (s->insideOctree(childCorner1, childCorner2, origin))
//                 {
//                     // One object might be in two octants, but the object can only be removed the parent's ArrayList once.
//                     // That's why I use a set here. Also its fast.

//                     rem.insert(s);
//                     octList[i].insert(s);
//                 }
//             }
//         }

//     // preallocate space
//     children.reserve(8);

//     // creating the children octrees once objects have been determined to lie within them
//     for (int i = 0; i < 8; i++)
//     {
//         // printf("Octant %d has %d obs in it\n", i, octList[i].size());
//         // only create a child if it will have any solids in it
//         if (octList[i].size() > 0)
//         {
//             // std::cout << "Created child node. obs in child = " << octList[i].size() << std::endl;
//             children[i] = new Octree(this, octantBounds[i][0], octantBounds[i][1][0] - octantBounds[i][0][0], octList[i], counter);
//         }
//         else
//         {
//             children[i] = nullptr;
//         }
//     }

//     // long long int t1 = getTime();

//     // remove all the solids that were placed into children octants from this octant.
//     //(if a solid is in a child, no reason to store it in this octant)
//     // Using a set for the removal data structure as well as the solids data structure was by far the fastest
//     // configuration I found.
//     for (auto it = rem.begin(); it != rem.end(); it++)
//     {
//         solids.erase(*it);
//     }

//     // The code commented out below code is for validation and testing optimizations. You can ignore it.

//     // long long int t2 = getTime();
//     // counter += (t2 - t1);
//     // make sure we didn't mistakenly discard any solids or duplicate something
//     // std::unordered_set<Solid *> total;
//     // validateObjectCount(total);
//     // obsContained = total.size();
//     // if (total.size() != objectsInOctant.size())
//     // {
//     //     printf("something went wrong %d\n", (total.size() - objectsInOctant.size()));
//     // }

//     // if (total.size() == 0)
//     // {
//     //     printf("This octant is useless\n");
//     // }

//     // std::unordered_set<Solid *> childrenObs;
//     // getObsInChildren(childrenObs);
//     // for(Solid* s : childrenObs)
//     //     allChildObs.push_back(s);

//     // std::cout << "Finished constructor. obs in octant = " << solids.size() << std::endl;
// }

// // Function to test runtime optimizations.
// // When called at the root of the Octree, it will return the number of octrees that
// // theoritically could be replace by a child node, without disturbing any data.
// // Only true if this octant if
// //   a) Doesn't have any solids in it
// //   AND
// //   b) Only has 1 child octant with solids in it
// int Octree::countReplaceable()
// {
//     int count = 0;
//     int numChildren = 0;
//     bool replaceable = true;
//     if (solids.size() > 0)
//         replaceable = false;

//     for (int i = 0; i < 8; i++)
//     {
//         if (children[i] != nullptr)
//         {
//             numChildren++;
//             count += children[i]->countReplaceable();
//         }
//     }

//     if (numChildren < 2 && solids.size() == 0)
//         replaceable = true;
//     if (replaceable)
//         return count + 1;
//     else
//         return count;
// }

// // makes sure that the number of objects in this octant plus the number of objects in all
// // children (recursive) equals the number of objects that was originally passed to this Octree node.
// void Octree::validateObjectCount(std::unordered_set<Solid *> &tot)
// {
//     for (Solid *s : solids)
//         tot.insert(s);

//     for (int i = 0; i < 8; i++)
//     {
//         if (children[i] != nullptr)
//         {
//             children[i]->validateObjectCount(tot);
//         }
//     }
// }

// // Populates tot with all the children in this octant or its children (recursive)
// void Octree::getObsInChildren(std::unordered_set<Solid *> &tot)
// {
//     for (std::unordered_set<Solid *> childrenObs : octList)
//         for (Solid *s : childrenObs)
//             tot.insert(s);

//     for (int i = 0; i < 8; i++)
//     {
//         if (children[i] != nullptr)
//         {
//             children[i]->getObsInChildren(tot);
//         }
//     }
// }

// // calculates the minimum distance a Solid is in this octant (or its children) to the camera location.
// // Used for runtime optimization
// double Octree::calcMinDistanceFromCam(Eigen::Vector3d &e)
// {
//     if (distFromCam == -1)
//     {
//         // std::cout << "here" << std::endl;
//         double dist = 10000000000;
//         for (Solid *s : solids)
//             dist = std::min(dist, s->calcDistanceToCam(e));

//         for (int i = 0; i < 8; i++)
//         {
//             if (children[i] != nullptr)
//             {
//                 dist = std::min(dist, children[i]->calcMinDistanceFromCam(e));
//             }
//         }

//         distFromCam = dist;

//         // sort this octant's children by dist to camera
//         // std::sort(children.begin(), children.end(), comparator());
//         // for (Octree *o : children)
//         // {
//         //     printf("dist = %f\n", o->distFromCam);
//         // }
//     }
//     return distFromCam;
// }

// pixelInfo Octree::intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT, double _nearest, long long int &timer, long int &counter, long int &triCounter)
// {
//     // counter++;
//     if (distFromCam == -1)
//     {
//         distFromCam = std::max(0.0, ((boundingSphere->pos - e).norm() - boundingSphere->radius));
//     }

//     //  first check if the ray intersects this octant
//     if (boundingSphere->intersect(rayDir, e, minT, maxT) >= maxT)
//         return pixelInfo{nullptr, maxT};

//     // setup vars for the intersection tests
//     double nearest = _nearest;
//     double intersectionS = maxT;
//     Solid *closest = nullptr;
//     pixelInfo info, possibleInfo;

//     // long long int t1 = getTime();

//     // first check the solids that are directly in this octant
//     for (Solid *s : solids)
//     {
//         if (s->distToCam < nearest)
//         {
//             intersectionS = s->intersect(rayDir, e, minT, maxT);

//             if (nearest > intersectionS)
//             {
//                 nearest = intersectionS;
//                 closest = s;
//             }
//             // triCounter++;
//         }
//     }
//     // long long int t2 = getTime();
//     // timer += (t2 - t1);

//     //  Now check if the ray intersects any of the child octants.
//     for (int i = 0; i < 8; i++)
//     {
//         if (children[i] != nullptr) // make sure the child exists
//         {
//             if (nearest > children[i]->distFromCam) // check if its possible that this child yeilds a closer intersection that what we already have
//             {
//                 possibleInfo = children[i]->intersect(rayDir, e, minT, maxT, nearest, timer, counter, triCounter);
//                 intersectionS = possibleInfo.t;
//                 if (nearest > intersectionS)
//                 {
//                     nearest = intersectionS;
//                     closest = possibleInfo.nearest;
//                 }
//             }
//         }
//     }

//     // below is some code that I wrote to test optimizations. You can ignore it.

//     // TODO: calculate if this cube's bounding circle is larger than one pixel using math
//     //  if we still havn't found an intersection yet it's because the bounding sphere is too small, so we'll simply check
//     //  all the objects in all the children.
//     //  if (closest == nullptr && size < .005) //Use math to compare to size something more reasonable.
//     //  {
//     //      // std::unordered_set<Solid *> childrenObs;
//     //      // getObsInChildren(childrenObs);
//     //      for (Solid *s : allChildObs)
//     //      {
//     //          intersectionS = s->intersect(rayDir, e, minT, maxT);

//     //         if (nearest > intersectionS)
//     //         {
//     //             nearest = intersectionS;
//     //             closest = s;
//     //         }
//     //     }
//     // }

//     // if (boundingSphere->radius < .001)
//     //     printf("small sphere inter numObs = %d, total = %d\n", solids.size(), obsContained);
//     // if (boundingSphere->radius < .1 && nearest != maxT)
//     //     printf("small sphere inter size = %f t=%f\n", size, nearest);

//     return pixelInfo{closest, nearest};
// }

// Octree::~Octree()
// {
//     // std::cout << "Destructor" << std::endl;

//     for (int i = 0; i < 8; i++)
//     {
//         delete[] octantBounds[i];

//         if (children[i] != nullptr)
//             delete children[i];
//     }

//     delete boundingSphere;
// }

// // helper function for populating the Octrees' bounding vectors
// Eigen::Vector3d *Octree::makeBounds(double x, double y, double z, double size)
// { // makes the bounds based on origin x,y and the size
//     Eigen::Vector3d *bounds = new Eigen::Vector3d[2];

//     Eigen::Vector3d frontTopRight, backBottomLeft;

//     // int[][] b = new int[3][2];
//     frontTopRight << x, y, z;
//     backBottomLeft << x + size, y + size, z + size;
//     bounds[0] = frontTopRight;
//     bounds[1] = backBottomLeft;

//     return bounds;
// }

// // Populates the Octrees' bounding vectors
// void Octree::setupOctantBounds()
// {
//     // std::cout << "Setting up octant bounds" << std::endl;

//     double x = origin[0];
//     double y = origin[1];
//     double z = origin[2];

//     // far octants first
//     octantBounds[0] = makeBounds(x - size / 2, y - size / 2, z - size / 2, size / 2); // NW
//     octantBounds[1] = makeBounds(x, y - size / 2, z - size / 2, size / 2);            // NE

//     octantBounds[2] = makeBounds(x - size / 2, y, z - size / 2, size / 2); // SW
//     octantBounds[3] = makeBounds(x, y, z - size / 2, size / 2);            // SE

//     // close octants now (changing z)
//     octantBounds[4] = makeBounds(x - size / 2, y - size / 2, z, size / 2); // NW
//     octantBounds[5] = makeBounds(x, y - size / 2, z, size / 2);            // NE

//     octantBounds[6] = makeBounds(x - size / 2, y, z, size / 2); // SW
//     octantBounds[7] = makeBounds(x, y, z, size / 2);            // SE
//     // std::cout << "Finished setting up octant bounds" << std::endl;
// }

// std::string Octree::toString(int spacer)
// {
//     // std::cout << "in tostring1 " << std::endl;

//     std::string data = "";
//     std::string space = "";
//     for (int i = 0; i < spacer * 4; i++)
//     {
//         space += " ";
//     }
//     if (solids.size() > 0)
//         data += space + "Level " + std::to_string(spacer) + ", Size = " + std::to_string(size) + " with " + std::to_string(solids.size()) + " objects\n" + space + "\n";
//     for (int i = 0; i < 8; i++)
//     {
//         if (children[i] != nullptr)
//         {
//             data += children[i]->toString(spacer + 1);
//         }
//     }
//     // std::cout << "in tostring " << std::endl;

//     return data;
// }

// // the rest of this file contains misc code I don't want to delete yet.

// // code to print info about this octant and its children.
// //  if (parent == nullptr)
// //  {
// //      for (int child = 0; child < 8; child++)
// //      {
// //          if (children[child] != nullptr)
// //          {
// //              printf("child = %d\n", child);
// //              for (int oc = 0; oc < 8; oc++)
// //              {
// //                  printf("Octant = %d\n", oc);

// //                 for (int j = 0; j < 3; j++)
// //                 {
// //                     for (int i = 0; i < 2; i++)
// //                     {
// //                         printf("%.0f ", children[child]->octantBounds[oc][i][j]);
// //                     }
// //                     printf("\n");
// //                 }
// //             }
// //         }
// //     }
// // }

// // Eigen::Vector3d corner1, corner2; //eMinusPos is computed once per frame
// // std::vector<Octree*> children;
// // std::vector<Solid*> objects;
// // double size;
// // double C; //C is computed once per frame
// // double radius;
// // bool first = true;

// // Octree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, std::vector<Solid*> solids, int minSize) : corner1(corner1), corner2(corner2)
// // {
// //     size = (corner1 - corner2).norm();

// //     //if this node is already at the minimum size store the rest of solids in this node.
// //     if(size < minSize)
// //         objects = solids;
// // }

// // void buildOctree(std::vector<Solid*> solids, int minSize){
// //     children = std::vector<Octree*>(8);
// //     std::vector<std::vector<Solid*>> childNodeSolids;

// //     for(int i = 0; i < 8; i++){
// //         Eigen::Vector3d childCorner1;
// //         Eigen::Vector3d childCorner2;

// //         for(Solid* s : solids){
// //             if(s->insideOctree(childCorner1, childCorner2)){
// //                 childNodeSolids[i].push_back(s);
// //             }
// //         }
// //     }
// // }

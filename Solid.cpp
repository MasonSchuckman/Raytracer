// #include "Solid.h"
// #include "Octree.h"

// Solid::Solid(ObjProps props) : properties(props)
// {
//     properties.setColor();
// }

// Solid::Solid(ObjProps props, std::vector<Eigen::Vector3d> verts) : properties(props), verts(verts)
// {
//     properties.setColor();
// }



// std::string Solid::toString()
// {
//     // std::stringstream s;
//     // s << verts;
//     // return s.str();
//     return "Solid has " + std::to_string(verts.size()) + " verticies.";
// }

// double Solid::calcDistanceToCam(Eigen::Vector3d &e)
// {
//     return -1;
// }

// double Solid::intersect(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, double minT, double maxT)
// {
//     std::cout << "base class intersect\n";
//     return maxT;
// }

// Eigen::Vector4d Solid::raytrace(Eigen::Vector3d rayDir, Eigen::Vector3d e, double minT, double maxT, int reflectionsRemaining, std::vector<Solid *> &solids, Solid *parent)
// {
//     std::cout << "base class raytrace\n";
//     return Eigen::Vector4d{0, 0, 0, 0};
// }

// bool Solid::insideOctree(Eigen::Vector3d corner1, Eigen::Vector3d corner2, Eigen::Vector3d center)
// {
//     std::cout << "base class inside octree\n";
//     return false;
// }

// // checks if point e is in shadow by checking LOS with all the lights in the scene.
// // bool Solid::inShadow(Octree* root, Eigen::Vector3d &e, std::vector<Light> &lights)
// // {
// //     std::cout << "base class shadow\n";
// //     return false;
// // }
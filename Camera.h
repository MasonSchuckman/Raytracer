#include "Entity.h"
#include "Solid.h"
#include "Octree.h"
#include "Light.h"
#include "Intersecter.h"
#include "LightCam.h"
#include <utility>
#include <map>

#ifndef Camera_H
#define Camera_H

#define PI 3.14159
#define rad PI / 180.0

class Camera : public Entity
{
public:
    int numLightCamSamples = 7;
    // IDEA: I could use a map where key = (solid, light) value = camera that corresponds to that.
    // The pair is the transparent solid, int num of the light
    std::map<std::pair<Solid *, int>, LightCam *> lightCams;
    int setupLightCams = 0;

    // 0 = Octree<Solid*>, 1 = no Octree<Solid*> (basic loop thru all solids method)
    // Not proud of how I'm implementing this flexibility but I don't have all day to make it pretty lol
    int intersectionMethod = 0;
    Intersecter basic;

    // how much light is added to every solid
    Eigen::Vector3d ambientLight = {50 / 255.0, 50 / 255.0, 50 / 255.0};

    // angle is fov, d is the distance from the camera to the target, h is the width of a pixel.
    double angle, d, aspectRatio, distToEdge, pixWidth, l, t;
    double selfIntersectBias = .01;
    int resx, resy;
    // u,v,w constitute our coordinate system.
    Eigen::Vector3d u, v, w;

    // not a huge fan of using from and at here, but doing it for convienience for now.
    Eigen::Vector3d from, at, e;

    Eigen::Vector3d backgroundColor;

    // used for timers
    long long int TIME;

    // constructoring using initializer list for parent class and this class
    Camera(double angle, int resx, int resy, double x, double y, double z) : Entity(x, y, z), angle(angle), resx(resx), resy(resy), from({x, y, z}){};

    ~Camera()
    {
        for (auto it = lightCams.begin(); it != lightCams.end(); it++)
            delete it->second;
    }
    // convenience function
    // void printVec(Eigen::Vector3d v)
    // {
    //     printf("[%f, %f, %f]\n", v[0], v[1], v[2]);
    // }

    // computes the coordinate system for the camera for a given position and target.
    // also precomputes useful info such as pixelWidth
    void computeCoordinateSystem(Eigen::Vector3d from, Eigen::Vector3d at, Eigen::Vector3d up, Eigen::Vector3d b)
    {
        // set the background color
        backgroundColor = b;

        w = (from - at).normalized(); // n
        u = up.cross(w).normalized(); // u
        v = w.cross(u).normalized();  // v

        aspectRatio = resx / resy;

        // compute distance from camera to target
        d = (from - at).norm();

        // alias to keep consistency with book and lecture
        e = from;

        // following the textbook, section 4.3.1
        // compute the distance from the center of the image to the top edge
        distToEdge = d * tan(angle / 2 * rad);

        // compute how large each pixel is
        pixWidth = 2 * distToEdge / resx; // we need a 2 here since we need to account for both directions

        // using the pixel width and the width of the image, we can find the edges of the image.
        // we'll then use the edges of the image as the starting point for our rays, and offset them
        // using the calculated pixel width.
        // need to offset with .5 here so we shoot rays in the middle of pixels.
        l = -distToEdge + 0.5 * pixWidth;              // left edge of the image
        t = distToEdge * aspectRatio - 0.5 * pixWidth; // top edge of image

        // used for measuring how long some functions take
        TIME = 0;
    }

    double getNormedRand()
    {
        double percision = 1000;
        return (rand() % (int)percision) / percision;
    }

    bool refraction = true;
    bool DoF = false;
    int numSamples = 1; // used for stratified sampling
    double aperture = .5;
    bool phong = false;
    // Either solids or root will go unused due to how I'm implementing some flexibility in how the intersections are done.
    // This function casts the "initial" ray for a pixel, then it calls recursiveCastRay to deal with reflections and refractions.
    Eigen::Vector3d castRay(int i, int j, Octree<Solid *> *root, std::vector<Solid *> &solids, std::vector<Light> &lights)
    {
        DoF = aperture > 0;

        // For DoF keep w fixed, move u and v.
        int maxReflections = 5;


        Eigen::Vector3d overallColor = {0, 0, 0};

        // loop for pixel jitter. Doesn't seem to be working right now.
        for (int si = 0; si < numSamples; si++)
        {
            for (int sj = 0; sj < numSamples; sj++)
            {

                double randX = 0;
                double randY = 0;
                double spw = pixWidth / numSamples; // sub pixel width

                // calculate the direction of the ray
                double p, q, r;

                // Logic for stratified sampling:
                /*
                For n = 1, j covers a distance of pixWidth.
                For n = 2, each subsample will cover a distance of pixWidth / 2
                For for n = k, j_m's assosciated distance will be (pixWidth / k) units big
                Subpixel width = spw = 1 / numSamples.
                The left most part of pixel is at j - 1/2 pixelWidth. So j_m = (j - (.5 * PW) + m * spw + .5 * spw.
                */

                // if we're doing stratified we need to slightly offset i and j.
                if (numSamples > 1)
                {
                    double percision = 1000;
                    randX = getNormedRand(); // random float between 0 and 1
                    randY = getNormedRand(); // random float between 0 and 1

                    // center the random vals on 0.
                    randX -= .5;
                    randY -= .5;
                    // now the rands are in [-0.5, 0.5].

                    // find edges of pixel
                    double edgeX = (j - .5) * pixWidth;
                    double edgeY = (i - .5) * pixWidth;

                    // calculate center of subpixel
                    double j_m = edgeX + sj * spw + .5 * spw;
                    double i_m = edgeY + si * spw + .5 * spw;
                    // printf("j = %d, j_m = %f, edgeX = %f\n", j, j_m, edgeX);
                    //  scale our random values by the sub pixel width
                    randX *= spw;
                    randY *= spw;

                    // turn off randomness
                    //randX = 0;
                    //randY = 0;

                    // now we can add our random values to our subpixel centers to get a random sampling point within our subpixel region
                    j_m += randX;
                    i_m += randY;

                    // calculate p and q based off the random location in our subpixel
                    p = l + j_m; // x
                    q = t - i_m; // y
                }
                else
                {                         // non stratified sampling calc
                    p = l + j * pixWidth; // x
                    q = t - i * pixWidth; // y
                }
                r = -d; // always calculated the same whether or not we're doing stratified sampling

                Eigen::Vector3d rayDir = p * u + q * v + r * w;
                rayDir.normalize();
                // std::cout << "rayDir = " << rayDir << std::endl;

                Eigen::Vector3d from = e; // This will be modified if DoF is enabled.

                if (DoF)
                {
                    // Overall offset from center of the lens
                    Eigen::Vector3d depthOfFieldOffset = {0, 0, 0};
                    // Offset in the direction of u
                    Eigen::Vector3d uOffset = (getNormedRand() - .5) * u * aperture;
                    // Offset in the direction of v
                    Eigen::Vector3d vOffset = (getNormedRand() - .5) * v * aperture;
                    depthOfFieldOffset = uOffset + vOffset;

                    // Add the offset to the "from" location
                    from += depthOfFieldOffset;

                    // Now we need to correct the rayDir so it still passes through the targeted location
                    // Use the non-AA p and q
                    p = l + j * pixWidth; // x
                    q = t - i * pixWidth; // y
                    Eigen::Vector3d focusPoint = p * (u) + q * (v) + r * w + e;
                    rayDir = (focusPoint - from).normalized();
                    //rayDir = p * (u + uOffset) + q * (v + vOffset) + r * w;
                    //rayDir.normalize();
                }

                // accumulate the color from this sample. Divided by numSamples^2 so we take the average of all the samples.
                overallColor += recursiveCastRays(rayDir, from, root, solids, lights, maxReflections, 1.0) / (numSamples * numSamples);
            }
        }

        // make sure we don't go out of bounds
        // Maybe this should be at the end of recursiveCastRays() instead, but Idk.
        for (int i = 0; i < 3; i++)
        {
            overallColor[i] = std::min(1.0, overallColor[i]);
            overallColor[i] = std::max(0.0, overallColor[i]);
        }

        return overallColor;
    }

    Eigen::Vector3d recursiveCastRays(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, Octree<Solid *> *root,
                                      std::vector<Solid *> &solids, std::vector<Light> &lights, int maxReflections, double CoR)
    {
        // now that we have the ray info for this pixel, we can find what solid it intersects with by checking
        // against all solids.

        HitRecord hr[maxReflections + 1];
        Eigen::Vector3d color = castRay(rayDir, e, root, solids, lights, hr[0], 0);
        // color << 0,0,0;
        double coefOfReflection = CoR;

        // This loop takes care of all the reflections.
        for (int i = 0; i < maxReflections; i++)
        {
            // Make sure we have something to reflect off of. (The last hit was good)
            if (hr[i].nearest == nullptr)
                break;

            // Check if this ray will change the image significantly or not
            if (coefOfReflection < .01)
                break;

            // check for refraction
            if (refraction && hr[i].nearest->properties.T > 0)
            {
                // Calculate refraction direction, then call recursiveCastRays() using this ray.

                // declare vars
                double ior = hr[i].nearest->properties.index_of_refraction;
                Eigen::Vector3d normal = hr[i].normal;
                Eigen::Vector3d viewDir = -hr[i].rayDir; // already normalized
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

                // We need to calculate the discriminate first so we don't try to take the sqrt of a negative.
                double discriminate = 1 - (1 - (dot * dot)) * (ior * ior);
                double T = hr[i].nearest->properties.T;
                if (discriminate > 0)
                {
                    // calc refraction ray dir
                    Eigen::Vector3d refractionDir = (normal * (dot)-viewDir) * ior - normal * std::sqrt(discriminate);

                    // Cast refraction ray
                    color += T * recursiveCastRays(refractionDir, hr[i].impactPos, root, solids, lights, maxReflections - i - 1, T * coefOfReflection);
                }
                else
                {
                    Eigen::Vector3d internalReflectionDir = (hr[i].rayDir + 2.0 * -hr[i].rayDir.dot(normal) * normal).normalized();

                    // not sure if I should use Ks or T here
                    color += T * recursiveCastRays(internalReflectionDir, hr[i].impactPos, root, solids, lights, maxReflections - i - 1, T * coefOfReflection);
                    // printf("Total internal reflection\n");
                }
            }

            coefOfReflection *= hr[i].nearest->properties.Ks; // not sure if this should be *= or =.
            color += coefOfReflection * castRay(hr[i].reflectionDir, hr[i].impactPos, root, solids, lights, hr[i + 1], (i + 1));
        }

        // if(hr[0].nearest != nullptr && hr[1].nearest != nullptr && hr[0].nearest->TYPE() == hr[1].nearest->TYPE()){
        //     std::cout << hr[0].nearest->TYPE() << " got reflection from " << hr[1].nearest->TYPE() << std::endl;
        // }

        return color;
    }

    // assumes rayDir is already normalized.
    // returns the local color where the ray hits, or the background color if the ray doesn't intersect anything
    Eigen::Vector3d castRay(Eigen::Vector3d &rayDir, Eigen::Vector3d &e, Octree<Solid *> *root, std::vector<Solid *> &solids, std::vector<Light> &lights, HitRecord &hr, int boundNum)
    {

        double maxT = 10000000000;
        double minT = selfIntersectBias;
        double intersectionT = maxT;

        // Traverse the Octree<Solid*> looking for intersections.
        // Save info about the closest intersection.
        hr.nearest = nullptr;
        hr.t = maxT;

        if (intersectionMethod == 0)
            root->intersect(rayDir, e, minT, maxT, maxT, hr);
        else
        {
            basic.intersect(solids, rayDir, e, minT, maxT, hr);
        }

        Eigen::Vector3d localColor; // what we're returning

        // check we got a good hit
        if (hr.nearest == nullptr)
        {
            // if we didn't get a hit this pixel takes the background color
            localColor = backgroundColor;
        }
        else
        {
            localColor = computeLocalColor(hr, e, lights, root, solids);

            // The below code adds distance and direction markings to the render. This helped validate light refraction through spheres.
            //  double dist = hr.impactPos.norm();
            //  if ((int)dist % 5  == 0 && dist > .8)
            //  {
            //      localColor += ambientLight;
            //      if (hr.impactPos[0] > 0)
            //          localColor += ambientLight;
            //      if (hr.impactPos[1] > 0)
            //          localColor += ambientLight;
            //  }
            //  localColor += ambientLight;
        }

        return localColor;
    }

    Eigen::Vector3d computeLocalColor(HitRecord &hr, Eigen::Vector3d &camPos, std::vector<Light> &lights, Octree<Solid *> *root, std::vector<Solid *> &solids)
    {
        Solid *solid = hr.nearest;
        Eigen::Vector3d rayDir = hr.rayDir;
        Eigen::Vector3d e = hr.e;
        Eigen::Vector3d impactPos = hr.impactPos;
        Eigen::Vector3d normal = hr.normal;

        Eigen::Vector3d localColor{0, 0, 0};

        double maxT = 10000000000;
        double minT = selfIntersectBias;

        // placeholder vars
        long long int a;
        long int b, c;

        normal.normalize(); // this should be redundant but idk if somehow isn't already normalized. I should prob check.
        // loop over all the lights in the scene, summing their effects
        int lightnum = 0;
        for (Light l : lights)
        {
            // The ray coming from the light to the impact point
            // Make sure it's normalized
            Eigen::Vector3d rayDir = (l.pos - impactPos).normalized();

            // check for intersection of this ray with any solids along the ray starting at the impact point to the light.
            Eigen::Vector3d distToLight = (l.pos - impactPos); // if we reach the light before any other object we have LOS.

            // check if the object has Line of Sight to the light.
            double coefOfRefraction = .999999;
            bool occluded = true;

            if (refraction)
            {
                // If the only objects occluding this light are transparent, we will cast rays from the light at this position and see if we get close.
                bool onlyTransparentOccluding = true;
                Solid *closest = nullptr;

                occluded = root->checkOccluded3(solid, rayDir, impactPos, minT, distToLight.norm() - selfIntersectBias, onlyTransparentOccluding, closest); // THIS MIGHT CAUSE ISSUES. TODO: LOOK INTO THIS!

                if (occluded && onlyTransparentOccluding)
                {

                    std::pair<Solid *, int> pair(closest, lightnum);

                    // TODO: MAKE THIS (actually) THREAD SAFE! (As it is, I haven't had any issues with it, but in theory it could be thread unsafe.)
                    // Also, as it is, it's very inefficient since it's creating and storing data for a given light, solid pair for each thread running.
                    // Ideally, I would change it so this information is precomputed before rendering starts, and have each light,solid pair be assigned to
                    // a unique thread.

                    // if this is the first time we've encountered this pair we need to create the light cam.
                    if (lightCams.count(pair) == 0)
                    {
                        // printf("First\n");
                        double distToSolid = (((Sphere *)closest)->pos - l.pos).norm(); // NOTE: I need to account for two spheres inline with each other and light.
                        // the angle that will make the transparent sphere fill the view of the camera
                        double angleOfView = atan2(((Sphere *)closest)->radius, distToSolid) * 2.82; // the 2.82 is from 2 * sqrt(2).

                        // base the number of samples off of how wide the angle from the light to the edges of the sphere is
                        // double maxSamps = 700;
                        // int nSamples = std::max(std::min(maxSamps * (angleOfView / (3.14159 / 2)), maxSamps), 10.0); // limit nSamples to be in the range of [100, 500^2]. Max numsamples when angle of view > 90 degrees

                        // printf("angle = %f, nSamples = %d\n", (angleOfView * 180 / 3.14159), nSamples);

                        // printf("angle = %f, in deg = %f, dist = %f, rad = %f\n", angleOfView, (angleOfView * 180 / 3.14159), distToSolid, ((Sphere *)closest)->radius);
                        LightCam *cam = new LightCam(angleOfView, 1, 1, l.pos[0], l.pos[1], l.pos[2]);
                        cam->computeCoordinateSystem(l.pos, ((Sphere *)closest)->pos, Eigen::Vector3d{0, .5, 1}, backgroundColor);
                        cam->sphere = ((Sphere *)closest);
                        cam->lightCast(solid, impactPos, numLightCamSamples, root, solids);
                        // printf("size = %d\n", cam->impactLocations.size());

                        lightCams.insert({pair, cam});

                        setupLightCams++;
                    }

                    // lightCamera.computeCoordinateSystem(l.pos, ((Sphere *)closest)->pos, Eigen::Vector3d{0, 0, 1}, backgroundColor);
                    // coefOfRefraction = lightCamera.lightCast(solid, impactPos, nSamples, root, solids);
                    coefOfRefraction = lightCams.at(pair)->calcIntensity(impactPos, l.pos, pos, at, pixWidth); // not sure if I should pass pixWidth or SPW here.

                    // Shade this pixel if it is sufficiently bright.
                    if (coefOfRefraction > .01)
                    {
                        occluded = false;
                    }
                }
            }
           // else
            // {
            //     if (intersectionMethod == 0)
            //     {
            //         occluded = root->checkOccluded(solid, rayDir, impactPos, minT, distToLight.norm() - selfIntersectBias); // THIS MIGHT CAUSE ISSUES. TODO: LOOK INTO THIS!
            //     }
            //     else
            //     {
            //         occluded = basic.checkOccluded(solids, solid, rayDir, impactPos, minT, distToLight.norm() - selfIntersectBias);
            //     }
            // }
            // TODO: CHANGE THIS BACK
            if (!occluded) //&& coefOfRefraction != .9999)
            {
                // make sure this surface is at all reflective
                if (solid->properties.Ks > 0)
                {
                    localColor += coefOfRefraction * solid->computeSpecularComp(hr, impactPos, e, camPos, normal, l);
                    // printf("Specular comp : %f %f %f\n", localColor[0], localColor[1], localColor[2]);
                }

                // if this location has LOS to Light l, calculate l's influence on the local color
                localColor += coefOfRefraction * solid->computeDiffuseComp(impactPos, normal, l);

                // printf("Color:\n");
                // std::cout << localColor << std::endl;
            }
            else
            {
                // printf("Light blocked\n");
            }
            lightnum++;
        }

        return localColor;
    }

    // passing by reference so we don't copy the vector each time we want to render a pixel.
    // returns the color and distance of the first solid hit
    pixelInfo castRay(int i, int j, std::vector<Solid *> &solids)
    {
        // calculate the direction of the ray
        double p, q, r;
        p = l + j * pixWidth; // x
        q = t - i * pixWidth; // y
        r = -d;

        Eigen::Vector3d rayDir = p * u + q * v + r * w;
        rayDir.normalize();
        // std::cout << "rayDir = " << rayDir << std::endl;

        // now that we have the ray info for this pixel, we can find what solid it intersects with by checking
        // against all solids.

        double maxT = 10000000000;
        double minT = 0;
        double intersectionT = maxT;

        Solid *closest = nullptr;

        // loop over every solid in the scene and check for intersection
        // save info about the closest intersection.
        for (Solid *s : solids)
        {
            double intersectionS = s->intersect(rayDir, from, minT, intersectionT);
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
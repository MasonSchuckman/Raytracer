#include "Entity.h"
#include "Solid.h"
#include "Octree.h"
#include "Light.h"
#include "Intersecter.h"

#ifndef LightCam_H
#define LightCam_H

#define PI 3.14159
#define rad PI / 180.0

class LightCam : public Entity
{
private:
    long long int getTime()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        return ms.count();
    }

public:
    double maxDistFromZero = 0;
    double totalEnergy = 0;
    Octree<Eigen::Vector3d *> *impacts;
    std::map<Eigen::Vector3d *, double> coefMap;
    // stores where every ray impacted
    std::vector<Eigen::Vector3d *> impactLocations;
    std::vector<double> coefs;
    Sphere *sphere;
    int nSamples = 1;
    double spw;
    // 0 = Octree, 1 = no Octree (basic loop thru all solids method)
    // Not proud of how I'm implementing this flexibility but I don't have all day to make it pretty lol
    int intersectionMethod = 0;
    Intersecter basic;

    // how much light is added to every solid
    Eigen::Vector3d ambientLight = {5 / 255.0, 5 / 255.0, 5 / 255.0};

    // angle is fov, d is the distance from the camera to the target, h is the width of a pixel.
    double angle, d, aspectRatio, distToEdge, pixWidth, l, t;
    double selfIntersectBias = .000001;
    int resx, resy;
    // u,v,w constitute our coordinate system.
    Eigen::Vector3d u, v, w;

    // not a huge fan of using from and at here, but doing it for convienience for now.
    Eigen::Vector3d from, at, e;

    Eigen::Vector3d backgroundColor;

    // used for timers
    long long int TIME;

    // constructoring using initializer list for parent class and this class
    LightCam(double angle, int resx, int resy, double x, double y, double z) : Entity(x, y, z), angle(angle), resx(resx), resy(resy), from({x, y, z}){};

    ~LightCam()
    {
        // delete impacts; //this was throwing an error :/
        for (Eigen::Vector3d *v : impactLocations)
            delete v;
    }

    // convenience function
private:
    void printVec(Eigen::Vector3d v)
    {
        printf("[%f, %f, %f]\n", v[0], v[1], v[2]);
    }

public:
    // computes the coordinate system for the camera for a given position and target.
    // also precomputes useful info such as pixelWidth
    void computeCoordinateSystem(Eigen::Vector3d from, Eigen::Vector3d at, Eigen::Vector3d up, Eigen::Vector3d b)
    {

        // set the background color
        backgroundColor = b;

        w = (from - at).normalized(); // n
        u = up.cross(w).normalized(); // u
        v = w.cross(u).normalized();  // v

        // printVec(w);
        // printVec(u);
        // printVec(v);

        aspectRatio = resx / resy;

        // compute distance from camera to target
        d = (from - at).norm();

        // alias to keep consistency with book and lecture
        e = from;

        // following the textbook, section 4.3.1
        // compute the distance from the center of the image to the top edge
        distToEdge = d * tan(angle / 2);

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
    int maxC = 0;
    // calculates the intensity at pos based on the previously calculated impact locations

    double energyPerRay = 0;
    int maxTot = 5;
    double calcIntensity(Eigen::Vector3d pos, Eigen::Vector3d lightPos, Eigen::Vector3d camPos, Eigen::Vector3d camAt, double camPixWidth)
    {
        double smoothingFactor = 1.5;
        double camDist = (pos - camPos).norm();
        double camToAt = (camPos - camAt).norm();
        double viewCameraPixelSpacing = camPixWidth * camDist / camToAt;

        double dist = (pos - lightPos).norm();
        double sphereDistToLight = (lightPos - sphere->pos).norm();
        double maxDist = spw * (dist / sphereDistToLight);
        maxDist *= smoothingFactor; // this is for smoothing of the result. The extra brightness is accounted when we divide by expected

        // double maxDist = spw * .1;
        // maxDist = .01;
        
        // printf("max = %f\n", maxDist);

        // maxDist = .05;
        int c = 0;

        double total = 0;
        int i = 0;
        // for (Eigen::Vector3d* v : impactLocations)
        // {
        //     double dist = ((*v) - pos).norm();
        //     if (dist < maxDist)
        //         c++;
        //     total += coefs[i] * (1.0 / std::pow(1 + dist, 2));
        //     //}
        //     // total += coefs[i] * (1.0 / std::pow((1 + dist), 2));
        //     // total += coefs[i] * (1.0 / std::pow((dist + pixWidth / 2), 2));
        //     i++;
        // }
        Eigen::Vector3d *loc = new Eigen::Vector3d(pos);
        total = getIntensity(impacts, maxDist, loc, c);
        delete loc;

        double expected = (maxDist * maxDist) / (spw * spw); // maybe divide maxDist by dist here
        // if (total < expected){
        //     printf("total = %f, expected = %f, pos:", total, expected);
        //     printVec(pos);
        // }
        //  if(c > 0 && (rand() % 400) == 0){
        //     printf("c = %d, expected = %f, maxDist = %f, spacing = %f\n", c, expected, maxDist, viewCameraPixelSpacing);
        //     printf("spw = %f, dist = %f, sphere dist = %f, maxDist = %f\n", spw, dist, sphereDistToLight, maxDist);

        //  }
        // double expected = 1;
        //   if(c > 0)
        //   {
        //       // if(c > 1300)
        //       // {
        //       //     printf("c = %d, total = %f\n", c, total);
        //       // }
        //       total = (c / 400.0);
        //   }
        //   total /= (impactLocations.size() * impactLocations.size());
        //  total /= (nSamples * nSamples);
        // total /= expected;
        // total *= energyPerRay;
        // total *= 100;
        //  total *= expected;
        //   if(c > expected)
        //  total *= (c / expected);
        //   total *= c;
        //  if(total > maxTot){
        //      maxTot = total;
        //      printf("total = %d\n", maxTot);
        //  }
        //  if(c > maxC){
        //      maxC = c;
        //  if(rand() % 10000 == 0)
        //     printf("expected = %f, pixWidth = %f, maxDist = %f\n", expected, spw, maxDist);

        //     //printf("new max = %d\n", c);
        // }
        // total = std::min(total, 2.0);
        // if(total > 0)
        // total = 1;
        //return (total / (expected * smoothingFactor * smoothingFactor));
        return total / expected;
        //return total;
    }

    // Light casting experiment below----------------------
    int maxReflections_ = 20;
    double lightCast(Solid *target, Eigen::Vector3d targetPos, int numSamples, Octree<Solid *> *root, std::vector<Solid *> &solids)
    {
        totalEnergy = 3.14159 * sphere->radius * sphere->radius;
        energyPerRay = totalEnergy / (numSamples * numSamples);
        nSamples = numSamples;

        double intensity = 0;

        // loop for pixel jitter. Doesn't seem to be working right now.
        for (int si = 0; si < numSamples; si++)
        {
            for (int sj = 0; sj < numSamples; sj++)
            {

                double randX = 0;
                double randY = 0;
                spw = pixWidth / numSamples; // sub pixel width

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
                    randX = (rand() % (int)percision) / percision; // random float between 0 and 1
                    randY = (rand() % (int)percision) / percision; // random float between 0 and 1

                    // center the random vals on 0.
                    randX -= .5;
                    randY -= .5;
                    // now the rands are in [-0.5, 0.5].

                    // find edges of pixel
                    double edgeX = (0 - .5) * pixWidth;
                    double edgeY = (0 - .5) * pixWidth;

                    // calculate center of subpixel
                    double j_m = edgeX + sj * spw + .5 * spw;
                    double i_m = edgeY + si * spw + .5 * spw;
                    // printf("j = %d, j_m = %f, edgeX = %f\n", j, j_m, edgeX);
                    //  scale our random values by the sub pixel width
                    randX *= spw;
                    randY *= spw;

                    // DISABLE NOISE
                    randX *= 0;
                    randY *= 0;

                    // now we can add our random values to our subpixel centers to get a random sampling point within our subpixel region
                    j_m += randX;
                    i_m += randY;

                    // calculate p and q based off the random location in our subpixel
                    p = l + j_m; // x
                    q = t - i_m; // y
                }
                else
                {                         // non stratified sampling calc
                    p = l + 0 * pixWidth; // x
                    q = t - 0 * pixWidth; // y
                }
                r = -d; // always calculated the same whether or not we're doing stratified sampling

                Eigen::Vector3d rayDir = p * u + q * v + r * w;
                rayDir.normalize();

                // printf("\nStarting ray\n");
                //  printf("Ray origin: \n");
                //  printVec(e);
                //  printf("Ray dir: \n");
                // printVec(rayDir);
                //  printf("s pos: \n");
                //  printVec(sphere->pos);

                // std::cout << "rayDir = " << rayDir << std::endl;

                // accumulate the color from this sample. Divided by numSamples^2 so we take the average of all the samples.

                intensity += recursiveCastLightRays(target, targetPos, rayDir, e, root, solids, maxReflections_, 1.0, 1.0, false, false, 0) / (numSamples * numSamples);
            }
        }

        // make sure we don't go out of bounds
        // Maybe this should be at the end of recursiveCastRays() instead, but Idk.

        // Once our first light cast is finished we can construct the octree of impact locations
        setupLightOctree();

        return intensity;
    }

    void setupLightOctree()
    {
        // long long int t1 = getTime();

        double size = maxDistFromZero * 2 + 1; // side length of original octree. Octree<Solid*>is centered at (0,0,0) by default.
        printf("There are %d total impactLocs in the scene.\n", impactLocations.size());

        // Octree<Solid*>*root = new Octree(nullptr, Eigen::Vector3d{0 - size/2, 0 - size/2, 0 - size/2}, size, solids);

        // convert our solids vector to a set
        std::unordered_set<Eigen::Vector3d *> impactSet;
        for (Eigen::Vector3d *v : impactLocations)
            impactSet.insert(v);

        // have to set it here so we can do the print statements in the function rather than in main()
        int numOctrees = 0;
        long long int t1 = getTime();
        impacts = new Octree<Eigen::Vector3d *>(nullptr, Eigen::Vector3d{0 - size / 2, 0 - size / 2, 0 - size / 2}, size, impactSet, numOctrees);
        // printf("Impact locations:\n");
        // for(Eigen::Vector3d* v : impacts->allChildObs){
        //     if((int)v[0][2] == -8)
        //     printVec(*v);
        // }

        // root->calcMinDistanceFromCam(from);
        long long int t2 = getTime();
        printf("Octree construction took %lld ms\n", (t2 - t1));
        // printf("There are %lld light octrees!\n", numOctrees);
        // printf("There are %d replaceable octrees!\n", impacts->countReplaceable());
    }

    int missed = 0;
    double recursiveCastLightRays(Solid *target, Eigen::Vector3d targetPos, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, Octree<Solid *> *root,
                                  std::vector<Solid *> &solids, int maxReflections, double CoR, double coef, bool entered, bool otherRefract, int hitSpheres)
    {
        int hits = hitSpheres;
        // now that we have the ray info for this pixel, we can find what solid it intersects with by checking
        // against all solids.
        bool oRefract = otherRefract;
        HitRecord hr[maxReflections + 1];
        // printf("\nStarting ray");
        double intensity = castLightRay(target, targetPos, rayDir, e, root, solids, hr[0], 0, coef, entered, oRefract, hitSpheres);

        // NOTE: This creates false warnings when a glass ball is partially inside another solid.
        //  if(hr[0].nearest != nullptr && entered && hr[0].nearest->type() != 1){
        //      printf("prob bad\n");
        //      printVec(hr[0].impactPos);
        //      printVec(hr[0].e);
        //      std::cout << otherRefract << std::endl;
        //  }
        //  printf("\n%d\n", maxReflections);
        //   this camera might see the transparent object through another transparent object so check for that here
        if (hr[0].nearest == nullptr || (maxReflections == maxReflections_ && hr[0].nearest->type() != 1))
        {
            return 0;
        }
        if (hr[0].nearest->properties.T > 0)
            hits++;
        // if(hr[0].nearest != nullptr && maxReflections == maxReflections_&& hr[0].nearest == sphere){
        //     printf("started ray\n");
        // }

        // color << 0,0,0;
        double coefOfReflection = CoR;

        // This loop takes care of all the reflections.
        for (int i = 0; i < maxReflections; i++)
        {
            // Make sure we have something to reflect off of. (The last hit was good)
            if (hr[i].nearest == nullptr)
                break;

            coefOfReflection *= hr[i].nearest->properties.Ks; // not sure if this should be *= or =.

            // Check if this ray will change the image significantly or not
            if(coef < .01)
            break;

            if (coefOfReflection < .01 && hr[i].nearest->properties.T == 0)
                break;

            // check for refraction
            if (hr[i].nearest->properties.T > 0)
            {
                Sphere *s = ((Sphere *)hr[i].nearest);
                // printf("\n");
                //  Calculate refraction direction, then call recursiveCastRays() using this ray.

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
                    // printf("Depth = %d, entering solid at ", maxReflections);
                    // printVec(hr[i].impactPos);
                    // printf("And solid is at ");
                    // printVec(((Sphere*)hr[i].nearest)->pos);
                    if (entered)
                        printf("Double enter warning!\n");
                    ior = 1.0 / ior;

                    // printf("entering %d\n", maxReflections - i);
                }
                else
                {
                    if (!entered)
                        printf("Double exit warning!\n");
                    // printf("exiting %d, dot = %f, ref = %f\n", maxReflections - i, dot, coefOfReflection);
                    //   If we're exitting the surface we need to negate the normal vector and thus the dot product.
                    normal = -normal;
                    dot = -dot;
                }
                double T = hr[i].nearest->properties.T;
                // We need to calculate the discriminate first so we don't try to take the sqrt of a negative.
                double discriminate = 1 - (1 - (dot * dot)) * (ior * ior);
                // printf("discriminate = %f, dot = %f\n", discriminate, dot);
                if (discriminate > 0)
                {

                    // calc refraction ray dir
                    Eigen::Vector3d refractionDir = (normal * (dot)-viewDir) * ior - normal * std::sqrt(discriminate);
                    refractionDir.normalize();
                    // printf("Doing refraction, coef = %f, coef2 = %f\n", T, coef);
                    //  Cast refraction ray

                    // if (s != sphere)
                    //{
                    //  printf("\nFrom :\n");
                    //  printVec(hr[i].e);
                    //  printVec(hr[i].rayDir);
                    //  printf("refraction dir:\n");
                    //  printVec(refractionDir);
                    oRefract = true;

                    //}

                    intensity += T * recursiveCastLightRays(target, targetPos, refractionDir, hr[i].impactPos, root, solids, maxReflections - i - 1, coefOfReflection, coef * T, entering, oRefract, hits);
                }
                else
                {

                    Eigen::Vector3d internalReflectionDir = (hr[i].rayDir + 2.0 * -hr[i].rayDir.dot(normal) * normal).normalized();
                    // if (s != sphere)
                    // {
                    //     printf("\nFrom :\n");
                    //     printVec(hr[i].e);
                    //     printVec(hr[i].rayDir);
                    //     printf("reflection dir:\n");
                    //     printVec(internalReflectionDir);
                    // }
                    // not sure if I should use Ks or T here
                    // printf("Total internal reflection\n");
                    intensity += T * recursiveCastLightRays(target, targetPos, internalReflectionDir, hr[i].impactPos, root, solids, maxReflections - i - 1, coefOfReflection, coef * T, entering, oRefract, hits);
                }
            }

            // coefOfReflection *= (1 - hr[i].nearest->properties.T);

            double tempCoef = coefOfReflection;
            // printf("coefReflect = %f, coef = %f\n", tempCoef, coef);
            hr[i + 1].nearest = nullptr;
            // intensity += coefOfReflection * castLightRay(target, targetPos, hr[i].reflectionDir, hr[i].impactPos, root, solids, hr[i + 1], maxReflections, tempCoef, entered, oRefract);
        }

        // if(hr[0].nearest != nullptr && hr[1].nearest != nullptr && hr[0].nearest->TYPE() == hr[1].nearest->TYPE()){
        //     std::cout << hr[0].nearest->TYPE() << " got reflection from " << hr[1].nearest->TYPE() << std::endl;
        // }

        return intensity;
    }

    // returns the intensity of light at that pos
    double castLightRay(Solid *target, Eigen::Vector3d targetPos, Eigen::Vector3d &rayDir, Eigen::Vector3d &e, Octree<Solid *> *root, std::vector<Solid *> &solids, HitRecord &hr, int boundNum, double coef_, bool entered, bool refr, int hitSpheres)
    {

        double maxT = 10000000000;
        double minT = selfIntersectBias;
        double intersectionT = maxT;

        // Traverse the Octree looking for intersections.
        // Save info about the closest intersection.
        hr.nearest = nullptr;
        hr.t = maxT;
        if (coef_ > .05)
        {
            if (intersectionMethod == 0)
                root->intersect(rayDir, e, minT, maxT, maxT, hr);
            else
            {
                basic.intersect(solids, rayDir, e, minT, maxT, hr);
            }
        }

        if (hr.nearest == nullptr)
        {
            return 1.0 / 1000000.0;
        }

        // NOTE: This if causes false errors when a solid is inside another
        // verify that if its "entered" a refractive solid that is a sphere (since we've only implemented that so far.)
        // Without this check it seems like somehow "forward" rays were being cast (Check using the extended rays option in the simulator).
        // if (entered && hr.nearest->type() != 1)
        // {
        //     printf("hit %d spheres\n", hitSpheres);
        //     return 1.0 / 1000000.0;
        // }
        if (hr.nearest != nullptr) // Removing hr.nearest type check here would potentially allow glass balls to glow
        {
            // printf("intensity = %f, bound = %d\n ",coef_, boundNum);//printVec(hr.impactPos);
            // if(refr){
            //     printf("Refracted ray hit at ");
            //     printVec(hr.impactPos);
            // }

            Eigen::Vector3d *impactPos = new Eigen::Vector3d(hr.impactPos);

            impactLocations.push_back(impactPos);
            coefs.push_back(coef_ * (1.0 - hr.nearest->properties.T));
            coefMap.insert({impactPos, coef_ * (1.0 - hr.nearest->properties.T)});
            double distToOrigin = (hr.impactPos.norm());

            // if(distToOrigin < .5 && e[2] < 13){
            //     printf("close to origin. From = ");
            //     printVec(e);
            //     printf("Impact pos = ");
            //     printVec(hr.impactPos);

            // }

            maxDistFromZero = std::max(distToOrigin, maxDistFromZero);
            double dist = (hr.impactPos - targetPos).norm();
            return 1.0 / (1 + dist * dist);
        }
        else
            return 1.0 / 1000000.0;
        // }else
        //    return 1.0 / 1000000.0;
    }

    // checks distance from center of octree rather than if its actually inside
    bool insideOctree2(Eigen::Vector3d pos, Eigen::Vector3d center, double epsilon, double size)
    {
        return (pos - center).norm() < (size + epsilon);
        // bool vertexInside = true;
        //  loop over every dimension
        //  for (int dim = 0; dim < 3; dim++)
        //  {
        //      vertexInside = vertexInside && (corner1[dim] <= epsilon + pos[dim] && pos[dim] - epsilon <= corner2[dim]);
        //  }
        //  return vertexInside;
    }

    double getIntensity(Octree<Eigen::Vector3d *> *r, double maxDist, Eigen::Vector3d *loc, int &c)
    {
        if (r == nullptr)
            return 0;
        //  calculate intensity based on all pnts in this octant (and its children (recursive))
        double total = 0;
        if (r->size < maxDist * 2 || r->leaf)
        {
            // if (r->leaf){
            //     //printf("max dist = %f, r size = %f, obs = %d\n", maxDist, r->size, r->allChildObs.size());
            //     //printVec(*loc);
            // }
            int i = 0;
            for (Eigen::Vector3d *v : r->allChildObs)
            {

                double dist = ((*v) - (*loc)).norm();
                if (dist < maxDist)
                {
                    c++;
                    // total += coefMap.at(v) * (1.0 / std::pow(1 + dist, 2));
                    total += coefMap.at(v);
                    // if(v[0][1] < 0 && v[0][0] < 0){
                    //     printf("found, %f, %f\n", v[0][0], v[0][1]);
                    // }else if(loc[0][0] < 0 && loc[0][1] < 0){
                    //     printf("miss\n");
                    // }
                }
                i++;
            }
            return total;
        }
        else
        {

            // traverse down the Octree
            for (int i = 0; i < 8; i++)
            {
                double epsilon = maxDist * 0.1;
                if (insideOctree2(*loc, r->boundingSphere->pos, epsilon, r->size))
                {
                    // printf("recursion\n");
                    total += getIntensity(r->children[i], maxDist, loc, c);
                }
            }
            return total;
        }

        return 0;
    }
};

#endif
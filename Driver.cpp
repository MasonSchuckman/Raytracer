using namespace std;
#include "Octree.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "Camera.h"
#include "Light.h"
#include "Solid.h"
#include "Triangle.h"
#include "Sphere.h"
#include "Polygon.h"
#include <chrono>
#include <thread>

#include "PolygonalPatch.h"

// utility function adapted from https://stackoverflow.com/questions/19555121/how-to-get-current-timestamp-in-milliseconds-since-1970-just-the-way-java-gets
// used only for performance testing.
long long int getTime()
{
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return ms.count();
}

// where the nff data will be stored.
// Using the same names as nff for simplicity
Eigen::Vector3d b; // background color

// camera info
Eigen::Vector3d from; // Location of the camera
Eigen::Vector3d at;   // where the camera is looking
Eigen::Vector3d up;   // what direction is "up"
double camera_angle;
double hither;
int resx, resy; // resolution of the image (img width, img height)

int maxReflections;
int lightCamSamples = 150;

vector<Light> lights;
vector<Solid *> solids;

Octree<Solid *> *root;

void renderRows2(int stride, int id, Camera camera, double *pixels, double *ts);

// returns false if file was unopenable, true otherwise
bool parseInput(string filename);

void render(Camera camera);

void renderMultithreaded(Camera camera);

void clearVec();

void writeImage(double *pixels);

Octree<Solid *> *setupOctree();

// Debug function to print what we parsed from nff file.
void printNffData();

// convenience function so you don't have to add ".nff" to the end of the file
string addNff(string file);

long int totalIntersectionTests = 0;
long int triCounter = 0;
int numOctrees = 0;
long long int timer = 0;

bool shading = true; // project 2 setting
bool usingOctrees = true;
bool multithreaded = true;
bool randomizeColor = false;
bool storingPolygon = false;
bool AA = true;
const int numthreads = 2;

int numAASamples = 1;
double aperture = 0;
bool phong = false;

string inputFileName = "";
string outputFileName = "hide.ppm";
void parseArguments(int argc, char **argv);
void setPhong();

int main(int argc, char **argv)
{
    parseArguments(argc, argv);
    
    // Various options
    multithreaded = true;
    usingOctrees = true;
    randomizeColor = false;

    // // get the input filename from the user
    // string filename = "teapot-3.nff";
    // cout << "Input NFF filename" << endl;
    // cin >> filename;
    // filename = addNff(filename);
    // inputFileName = filename;

    // read the input file
    long long int t1 = getTime();
    parseInput(inputFileName);
    setPhong();
    long long int t2 = getTime();
    printf("Nff parsing and polygon construction took %lld ms\n", (t2 - t1));
    // printNffData();

    // setup the camera
    Camera camera(camera_angle, resx, resy, from[0], from[1], from[2]);
    // Extra credit parameters
    camera.numSamples = numAASamples;
    camera.aperture = aperture;
    camera.phong = phong;
    camera.computeCoordinateSystem(from, at, up, b);

    // Extra extra credit parameter (ignore)
    camera.numLightCamSamples = lightCamSamples;

    // construct the Octree
    if (usingOctrees)
    {
        root = setupOctree();
        camera.intersectionMethod = 0; // set the camera to use the Octree<Solid*>for intersections instead of the basic method
    }

    // render the image
    t1 = getTime();
    if (multithreaded)
        renderMultithreaded(camera);
    else
        render(camera);
    t2 = getTime();

    // print some stats
    printf("Render took %lld ms\n", (t2 - t1));
    if (timer != 0)
        printf("Intersection took %lld ms\n", timer);

    // print how many intersection calculations we had to do
    if (!usingOctrees)
        totalIntersectionTests = resx * resy * solids.size();

    // print more stats
    if (totalIntersectionTests != 0)
    {
        printf("Did %ld total intersection calculations\n", (totalIntersectionTests + triCounter));
        printf("Did %ld triangle calculations\n", triCounter);
        printf("%.3f% were bounding sphere calculations\n", (100 * (float)totalIntersectionTests / (float)(totalIntersectionTests + triCounter)));
    }

    // clear memory
    clearVec();

    if (usingOctrees)
        delete root;

    return 0;
}

void parseArguments(int argc, char **argv)
{
    printf("argc = %d\n", argc);
    int arg = 1;
    int pos = 0;
    while (arg < argc)
    {
        printf("arg = %d, char = %c\n", arg, argv[arg][pos]);

        switch (argv[arg][pos])
        {
        case '-':
        {
            pos++;
            break;
        }
        case 's':
        {
            numAASamples = atoi(argv[arg + 1]);
            arg += 2;
            pos = 0;
            break;
        }

        case 'a':
        {
            aperture = atof(argv[arg + 1]);
            arg += 2;
            pos = 0;
            break;
        }

        case 'p':
        {
            phong = true;
            arg++;
            pos = 0;
            break;
        }
        default:
            arg++;
            pos = 0;
            break;
        }
    }

    inputFileName = addNff(argv[argc - 2]);
    outputFileName = argv[argc - 1];
    printf("Arguments recieved:\nNum samples = %d\nAperture = %f\nPhong = %d\n", numAASamples, aperture, (phong) ? 1 : 0);
}

void setPhong()
{
    for (Solid *s : solids)
        s->phong = phong;
}

Octree<Solid *> *setupOctree()
{

    long long int t1 = getTime();

    // Pls don't make the scene bigger than this just to break it...thanks! (My lightcam octree has an implementation to fix this but
    // I don't have the energy to fix it here too).
    double size = 100; // side length of original octree. Octree<Solid*>is centered at (0,0,0) by default.
    printf("There are %d total solids in the scene.\n", solids.size());

    // Octree<Solid*>*root = new Octree(nullptr, Eigen::Vector3d{0 - size/2, 0 - size/2, 0 - size/2}, size, solids);

    // convert our solids vector to a set
    std::unordered_set<Solid *> solidsSet;
    for (Solid *s : solids)
        solidsSet.insert(s);

    // have to set it here so we can do the print statements in the function rather than in main()
    Octree<Solid *> *root = new Octree<Solid *>(nullptr, Eigen::Vector3d{0 - size / 2, 0 - size / 2, 0 - size / 2}, size, solidsSet, numOctrees);
    // root->calcMinDistanceFromCam(from);
    long long int t2 = getTime();
    printf("Octree construction took %lld ms\n", (t2 - t1));
    printf("There are %lld octrees!\n", numOctrees);
    printf("There are %d replaceable octrees!\n", root->countReplaceable());

    return root;
}

void renderRows(int start, int end, Camera camera, double *pixels, double *ts)
{
    bool testingReflections = false; // only "works" with spheres right now
    long int total = 0;
    long int last = 0;
    cout << "Started rendering" << endl;
    for (int i = start; i < end; i++) // rows
    {
        if (start == 0 && i % 128 == 0)
        {
            double percentDone = (i / ((double)end)) * 100;
            printf("Rendering... %.1f%\n", percentDone);
        }

        for (int j = 0; j < resx; j++)
        {
            // calculate the index for this pixel
            int offset = i * resx * 3 + j * 3;

            if (shading)
            {
                Eigen::Vector3d pixelColor = camera.castRay(i, j, root, solids, lights);

                pixels[offset + 0] = pixelColor[0] * 255;
                pixels[offset + 1] = pixelColor[1] * 255;
                pixels[offset + 2] = pixelColor[2] * 255;
            }
        }
    }

    total /= (resx * resy);
    if (total != 0)
        printf("average solids per pixel = %ld\n", total);
    // printf("intersect time = %lld ms\n", camera.TIME);
}

//(ignore for project 1)
// pseudo shader based on distance from the camera. Kinda works on simple scenes. Basically free to run.
void shade(double *pixels, double *ts)
{
    cout << "Started shading" << endl;
    double minT = 100000000000;
    double maxT = 0;
    for (int i = 0; i < resy; i++)
    {
        for (int j = 0; j < resx; j++)
        {
            // cout << ts[i * resx + j] << endl;
            // check that this pixel wasn't -1 (background, no intersection)
            if (ts[i * resx + j] > 0)
                minT = min(minT, ts[i * resx + j]);
            maxT = max(maxT, ts[i * resx + j]);
        }
    }
    double range = maxT - minT;
    cout << "range = " << range << endl;
    printf("max = %f, min = %f\n", (float)maxT, (float)minT);
    // scale all pixels using the range
    for (int i = 0; i < resy; i++)
    {
        for (int j = 0; j < resx; j++)
        {
            for (int color = 0; color < 3; color++)
            {
                if (ts[i * resx + j] != -1000)
                    pixels[i * resx * 3 + j * 3 + color] = (double)pixels[i * resx * 3 + j * 3 + color] * (1 - (ts[i * resx + j] - minT) / range);
            }
        }
    }
}

void normalizeImage(double *pixels)
{
    cout << "Started image normalization." << endl;
    double minT = 100000000000;
    double maxT = 0;
    for (int i = 0; i < resy; i++)
    {
        for (int j = 0; j < resx; j++)
        {
            for (int color = 0; color < 3; color++)
            {
                minT = std::min(minT, (double)pixels[i * resx * 3 + j * 3 + color]);
                maxT = std::max(maxT, (double)pixels[i * resx * 3 + j * 3 + color]);
            }
        }
    }
    double range = maxT - minT;
    cout << "range = " << range << endl;
    printf("max = %f, min = %f\n", (float)maxT, (float)minT);
    // scale all pixels using the range
    for (int i = 0; i < resy; i++)
    {
        for (int j = 0; j < resx; j++)
        {
            for (int color = 0; color < 3; color++)
            {
                pixels[i * resx * 3 + j * 3 + color] = 255 * (pixels[i * resx * 3 + j * 3 + color] - minT) / range;
            }
        }
    }
}

void renderMultithreaded(Camera camera)
{

    double *pixels = new double[resx * resy * 3]; // TODO: create this dynamically.
    double *ts = new double[resx * resy];         // TODO: create this dynamically.

    std::thread threads[numthreads];

    // width is how many rows each thread handles.
    int width = resy / numthreads;

    // Assign each thread a range of rows to work on.
    for (int i = 0; i < numthreads; i++)
    {
        int start = i * width;
        int end = start + width;
        if (i - 1 == numthreads)
            end = resy;
        // create and start the thread
        // threads[i] = std::thread(renderRows, start, end, camera, pixels, ts);

        // EXPIERMENTAL better multithreading by splitting the work more evenly (ONLY WORKS WHEN resy = k*numThreads)
        threads[i] = std::thread(renderRows2, numthreads, i, camera, pixels, ts);
    }

    // wait for the threads to finish
    for (int i = 0; i < numthreads; i++)
        threads[i].join();

    // normalizeImage(pixels);
    //  shade(pixels, ts);
    delete[] ts;

    writeImage(pixels);
    delete[] pixels;
}

// Assumes camera is already setup and all geometry we want to render is stored in the "solids" vector.
void render(Camera camera)
{
    double *pixels = new double[resx * resy * 3]; // TODO: create this dynamically.
    double *ts = new double[resx * resy];         // TODO: create this dynamically.

    cout << "Beginning render...\n";
    printf("There are %d objects in the scene.\n", solids.size());

    renderRows(0, resy, camera, pixels, ts);
    // shade(pixels, ts);
    delete[] ts;

    // for (int i = 1; i < resy; i *= 2)
    // {
    //     for (int j = 1; j < resy; j *= 2)
    //     {
    //         int offset = i * resx * 3 + j * 3;
    //         printf("pixel %d,%d = %d,%d,%d\n", i, j, pixels[offset + 0], pixels[offset + 1], pixels[offset + 2]);
    //     }
    // }

    writeImage(pixels);
    delete[] pixels;
}

void clearVec()
{
    for (int i = 0; i < solids.size(); i++)
    {
        delete solids[i];
    }
}

bool parseInput(string filename)
{
    bool simple = false;
    double minRadius = .1;
    ifstream datafile;
    datafile.open(filename);

    // break the file into individual lines
    string line;
    // break the file into individual "words". IE tokens seperated by spaces
    string word;

    if (datafile.is_open())
    {
        while (getline(datafile, line))
        {
            char lineType = line[0];
            // useful for reading in elements in a convenient way
            stringstream lineStream(line);
            ObjProps props;
            // cout << line << endl;
            //  check for polygonal patch before proceeding to other possibilities
            if ((line.length() > 1) && line[0] == 'p' && line[1] == 'p') // polygonal patch case
            {

                int numVerticies = 0;
                lineStream >> word >> numVerticies;

                // printf("num verts = %d\n", numVerticies);
                PolygonalPatch *pp = new PolygonalPatch(props, numVerticies);
                Eigen::Vector3d vertex;
                Eigen::Vector3d normal;

                // loop over all the verticies
                // Currently assumes perfect formatting of all verticies, and no spacing between lines
                // containing verticies. TODO: make this more robust.
                for (int v = 0; v < numVerticies; v++)
                {
                    lineStream.clear(); // we need to call .clear() here since we're reusing the string stream
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> vertex[0] >> vertex[1] >> vertex[2] >> normal[0] >> normal[1] >> normal[2];

                    // add the vertex to the list
                    pp->addVertex(vertex, normal);
                }

                // triangulate the polygon
                // poly->fanTriangles();
                pp->naiveEarClipping();

                if (storingPolygon)
                {
                    solids.push_back(pp);
                }
                else
                {
                    for (int i = 0; i < pp->triangles.size(); i++)
                    {
                        solids.push_back(pp->triangles[i]);
                    }
                    pp->triangles.clear();
                    delete pp;
                }
            }
            else // check other possibilities
            {
                switch (lineType)
                {
                case 'b':
                    // syntax to read the seperate tokens in the line into their respective places in a vector
                    lineStream >> word >> b[0] >> b[1] >> b[2];
                    break;

                // Personal testing variable
                case 'Q':
                    // syntax to read the seperate tokens in the line into their respective places in a vector
                    lineStream >> word >> maxReflections;
                    break;
                case 'Z':
                    // syntax to read the seperate tokens in the line into their respective places in a vector
                    lineStream >> word >> lightCamSamples;
                    break;
                case 'v':
                    // handle "from"
                    lineStream.clear(); // we need to call .clear() here since we're reusing the string stream
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> word >> from[0] >> from[1] >> from[2];

                    // handle "at"
                    lineStream.clear();
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> word >> at[0] >> at[1] >> at[2];

                    // handle "up"
                    lineStream.clear();
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> word >> up[0] >> up[1] >> up[2];

                    // handle "angle"
                    lineStream.clear();
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> word >> camera_angle;

                    // handle "hither"
                    lineStream.clear();
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> word >> hither;

                    // handle "resolution"
                    lineStream.clear();
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> word >> resx >> resy;

                    break;
                case 'l':
                {
                    // set a default color for the lights in case the nff doesn't contain color info for this light
                    Eigen::Vector3d light_color{{1, 1, 1}};
                    Eigen::Vector3d temp_pos;
                    lineStream >> word >> temp_pos[0] >> temp_pos[1] >> temp_pos[2];

                    // check if color info is included
                    if (!lineStream.eof())
                    {
                        Eigen::Vector3d temp_color;
                        lineStream >> word >> temp_color[0] >> temp_color[1] >> temp_color[2];
                        // override the default color
                        light_color = temp_color;
                    }

                    // add the light to the vector
                    lights.push_back(Light(light_color, temp_pos));

                    break;
                }
                case 'T':
                {

                    Triangle *tri = new Triangle(props);
                    Eigen::Vector3d vertex;
                    lineStream.clear(); // we need to call .clear() here since we're reusing the string stream
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> vertex[0] >> vertex[1] >> vertex[2];

                    tri->addVertex(vertex);

                    lineStream.clear();
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> vertex[0] >> vertex[1] >> vertex[2];

                    tri->addVertex(vertex);

                    lineStream.clear();
                    getline(datafile, line);
                    lineStream.str(line);
                    lineStream >> vertex[0] >> vertex[1] >> vertex[2];

                    tri->addVertex(vertex);
                    tri->finalize();
                    solids.push_back(tri);

                    break;
                }
                case 'p':
                {
                    int numVerticies = 0;
                    lineStream >> word >> numVerticies;

                    Polygon *poly = new Polygon(props, numVerticies);
                    Eigen::Vector3d vertex;
                    // loop over all the verticies
                    // Currently assumes perfect formatting of all verticies, and no spacing between lines
                    // containing verticies. TODO: make this more robust.
                    for (int v = 0; v < numVerticies; v++)
                    {
                        lineStream.clear(); // we need to call .clear() here since we're reusing the string stream
                        getline(datafile, line);
                        lineStream.str(line);
                        lineStream >> vertex[0] >> vertex[1] >> vertex[2];

                        // add the vertex to the list
                        poly->addVertex(vertex);
                    }

                    // triangulate the polygon
                    // poly->fanTriangles();
                    poly->naiveEarClipping();

                    // for(Triangle* tri : poly->triangles){
                    //     solids.push_back(tri);
                    // }
                    for (int i = 0; i < poly->triangles.size(); i++)
                    {
                        solids.push_back(poly->triangles[i]);
                    }
                    poly->triangles.clear();
                    delete poly;

                    // solids.push_back(poly);

                    break;
                }
                case 'f':
                {
                    double r, g, b, Kd, Ks, Shine, T, index_of_refraction;
                    lineStream >> word >> r >> g >> b >> Kd >> Ks >> Shine >> T >> index_of_refraction;
                    props = ObjProps{r, g, b, Kd, Ks, Shine, T, index_of_refraction};

                    break;
                }
                case 's':
                {
                    Eigen::Vector3d pos;
                    double r;
                    lineStream >> word >> pos[0] >> pos[1] >> pos[2] >> r;
                    // TODO: remove this random!
                    //  props.r = (rand() % 255) / 255.0;
                    //  props.g = (rand() % 255) / 255.0;
                    //  props.b = (rand() % 255) / 255.0;

                    if (!simple || r > minRadius)
                    {
                        Sphere *s = new Sphere(props, pos, r);
                        solids.push_back(s);
                    }
                    break;
                }
                default:
                    break;
                }
            }
        }
    }
    else
    {
        cout << "error opening file" << endl;
        return false;
    }

    if (randomizeColor)
    {
        for (Solid *s : solids)
        {
            s->properties.r = (rand() % 255) / 255.0;
            s->properties.g = (rand() % 255) / 255.0;
            s->properties.b = (rand() % 255) / 255.0;
        }
    }

    // once everything has been read in, set the lights' intensity
    for (int i = 0; i < lights.size(); i++)
    {
        lights[i].intensity = 1.0 / std::sqrt(lights.size());
    }

    return true;
}

void printNffData()
{
    cout << b << endl; // background color

    // camera info
    cout << "from " << from << endl; // Location of the camera
    cout << "at " << at << endl;     // where the camera is looking
    cout << "up " << up << endl;     // what direction is "up"
    cout << "camera angle " << camera_angle << endl;
    cout << "hither " << hither << endl;
    cout << "resolution " << resx << " " << resy << endl; // resolution of the image (img width, img height)

    cout << "Lights:" << endl;
    for (Light l : lights)
    {
        cout << l.toString() << endl;
    }
}

void writeImage(double *pixels)
{

    unsigned char *pixels_ = new unsigned char[resx * resy * 3];
    for (int i = 0; i < resy; i++)
    {
        for (int j = 0; j < resx; j++)
        {
            for (int color = 0; color < 3; color++)
            {
                pixels_[i * resx * 3 + j * 3 + color] = (int)pixels[i * resx * 3 + j * 3 + color];
            }
        }
    }

    FILE *f = fopen("hide.ppm", "wb");
    fprintf(f, "P6\n%d %d\n%d\n", resx, resy, 255);
    fwrite(pixels_, 1, resx * resy * 3, f);
    fclose(f);

    delete[] pixels_;
}

void renderRows2(int stride, int id, Camera camera, double *pixels, double *ts)
{
    bool testingReflections = false; // only "works" with spheres right now
    long int total = 0;
    long int last = 0;
    cout << "Started rendering" << endl;
    for (int i = id; i < resy; i += stride) // rows
    {
        if (id == 0 && i % 128 == 0)
        {
            double percentDone = (i / ((double)resy)) * 100;
            printf("Rendering... %.1f%\n", percentDone);
        }

        for (int j = 0; j < resx; j++)
        {
            // calculate the index for this pixel
            int offset = i * resx * 3 + j * 3;

            if (shading)
            {
                Eigen::Vector3d pixelColor = camera.castRay(i, j, root, solids, lights);

                pixels[offset + 0] = pixelColor[0] * 255;
                pixels[offset + 1] = pixelColor[1] * 255;
                pixels[offset + 2] = pixelColor[2] * 255;
            }
        }
    }

    total /= (resx * resy);
    if (total != 0)
        printf("average solids per pixel = %ld\n", total);
    // printf("intersect time = %lld ms\n", camera.TIME);
}

string addNff(string file)
{
    bool goodEnd = true;
    string nff = ".nff";
    for (int i = 0; i < 4; i++)
    {
        goodEnd = goodEnd && file[file.size() - 4 + i] == nff[i];
    }
    if (goodEnd)
        return file;
    else
        return file + nff;
}

Build command (instead of make):
g++ -I/usr/include/eigen3/ Driver.cpp -o driver -O3

General info:
You can run the program with ./driver [extra credit arguments here as seen in the prof's code] [input.nff].
You can set an output.ppm but it'll write to hide.ppm anyways since I forgot to change that.

You can mostly ignore the Octree.h and LightCam.h files. LightCam.h is entirely outside the scope of this project and was my attempt at caustics from refractive spheres. Octree.h is very simmilar to proj1 just with some extra functionality for this project.

My parser assumes there are no empty lines in between vertices in polygon sections of nff.

Multithreading is enabled by default in the program, but if you run the program and it doesn't work, try setting "numthreads = 1;" on line 80 of driver.cpp. Doubt it won't run though since I've tested on GL myself.


Extra credit:
Attempted all extra credit.
For the reflected portion of the teapot in refract.nff seems to be brighter than intended.

Statement of help:
I have not recieved any help from other students. The only help I recieved (other than the professor, TA, and the textbook), is clearly documented where I used it in the code with either a citation or a link to a webpage. Note: I didn't use any outside code for the project. I only used outside resources for ideas and help with some of the math.
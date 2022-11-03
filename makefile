
CXX = g++
CXXFLAGS = -O3 -I/usr/include/eigen3/

Driver: Driver.cpp
	 $(CXX) $(CXXFLAGS) Driver.cpp -o Driver


clean:
	rm *.o
	rm *~

run:
	./driver

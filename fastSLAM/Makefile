CPP=g++
CXXFLAGS="-g"

all: fastSLAM

fastSLAM: loMeasurement.o particle.o sitRobotMM.o

loMeasurement.o: loMeasurement.cpp
	$(CPP) $(CXXFLAGS) -c -o $@ $<

particle.o: particle.cpp
	$(CPP) $(CXXFLAGS) -c -o $@ $<

sitRobotMM.o: sitRobotMM.cpp
	$(CPP) $(CXXFLAGS) -c -o $@ $<


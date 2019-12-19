#compiler

OMPL_DIR = /usr/local
EIGEN_DIR = /home/avishai/Documents/eigen
KDL_DIR = /usr/local

CXX= g++
CXXFLAGS= -I${OMPL_DIR}/include -I${OMPL_DIR}/lib/x86_64-linux-gnu -I${KDL_DIR}/include  -I$(EIGEN_DIR) 
LDFLAGS=  -L${KDL_DIR}/lib -larmadillo -lorocos-kdl 

CPPSMP = kdl_class_comau3.cpp

all:
	$(CXX) main.cpp ${CPPSMP} -o m $(CXXFLAGS) $(LDFLAGS) -std=c++11	







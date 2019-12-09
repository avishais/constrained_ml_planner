#include "simulator.hpp"

int main (int argc, char** argv){
	ros::init(argc, argv,"ABB_simulator");
	ABB_simulator g;
	g.spin(15);
	
	return 0;
}
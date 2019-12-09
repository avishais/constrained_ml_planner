#include "projector.h"

int main (int argc, char** argv){
	ros::init(argc, argv,"ABB_projector");
	ABB_projector g;
	g.spin(15);
	
	return 0;
}
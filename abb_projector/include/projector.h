
#include "ros/ros.h"
#include <vector>
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
// #include "common_msgs_gl/SendDoubleArray.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include "kdl_class.h"
#include "abb_projector/project_srv.h"
#include "abb_projector/sample_srv.h"


#define ROBOTS_DISTANCE 900
#define ROD_LENGTH 300

using namespace std;

class ABB_projector : public kdl {

public:
	ABB_projector();
	void spin(int frequency = 100);

    bool callbackProject(abb_projector::project_srv::Request& req, abb_projector::project_srv::Response& res);
    bool callbackSample(abb_projector::sample_srv::Request& req, abb_projector::sample_srv::Response& res);

protected:
	ros::NodeHandle node_handle_;
    void initialize();

    ros::ServiceServer srv_project, srv_sample;	

}; 
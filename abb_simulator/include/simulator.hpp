
#include "ros/ros.h"
#include <vector>
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64MultiArray.h"
// #include "common_msgs_gl/SendDoubleArray.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <GL/glut.h>
#include "PQP.h"
#include "model.h"
#include "MatVec.h"
#include <iostream>
#include <unistd.h>
#include <string.h>

#define ROBOTS_DISTANCE 900
#define ROD_LENGTH 300

using namespace std;

typedef std::vector<std::vector<std::vector< double > > > Matrix_rod;
typedef std::vector<std::vector< double  > > Matrix_robo;

class ABB_simulator {
	// std::vector<double> finger_initial_offset_, finger_closing_position_, finger_opening_position_;
	// ros::ServiceClient srvclnt_send_gripper_commands_;
	// ros::ServiceClient srvclnt_read_gripper_data_, srvclnt_set_operating_mode_;

	// ros::Subscriber sub_pos_ref_, sub_vel_ref_;
	// ros::Publisher pub_pos_ref_, pub_vel_ref_;

	// bool velocity_mode_on_ = false;
	// bool initial_motor_pos_set_ = false;
	// std::vector<float> vel_cont_init_motor_pos_;
	// std::vector<float> motor_pos_ref_;
	// std::vector<double> vel_ref_;
    // std::vector<double> pos_ref_;

    PQP_Model base, link1, link2, link3, link4, link5, link6, EE, table, room, floor1;
    Model *base_to_draw, *link1_to_draw, *link2_to_draw, *link3_to_draw, \
    *link4_to_draw, *link5_to_draw, *link6_to_draw, *EE_to_draw;

    PQP_Model base2, link12, link22, link32, link42, link52, link62, EE2, rod, obs1, obs2, obs3;
    Model *base_to_draw2, *link1_to_draw2, *link2_to_draw2, *link3_to_draw2, \
    *link4_to_draw2, *link5_to_draw2, *link6_to_draw2, *EE_to_draw2, \
    *rod_to_draw, *table_to_draw, *obs1_to_draw, *obs2_to_draw, *obs3_to_draw, \
    *room_to_draw, *floor_to_draw;

    PQP_REAL R0[3][3],R1[3][3],R2[3][3],T0[3],T1[3],T2[3];
    PQP_REAL R3[3][3],R4[3][3],R5[3][3],T3[3],T4[3],T5[5];
    PQP_REAL R6[3][3],R7[3][3],T6[3],T7[3],T2_t[3],Ti[3];
    PQP_REAL M0[3][3],M1[3][3],M2[3][3],M3[3][3],M4[3][3];
    PQP_REAL M5[3][3],M6[3][3],M7[3][3];

    PQP_REAL R02[3][3],R12[3][3],R22[3][3],T02[3],T12[3],T22[3];
    PQP_REAL R32[3][3],R42[3][3],R52[3][3],T32[3],T42[3],T52[5];
    PQP_REAL R62[3][3],R72[3][3],T62[3],T72[3],T2_t2[3];
    PQP_REAL M02[3][3],M12[3][3],M22[3][3],M32[3][3],M42[3][3];
    PQP_REAL M52[3][3],M62[3][3],M72[3][3];

    PQP_REAL Mobs[3][3], Tobs[3];
    Matrix_rod RodStates;
    Matrix_robo RoboStates;

    int mode;
    double beginx, beginy;
    double dis = 2500.0, azim = 180.0, elev = 120.0;
    double ddis = 700.0, dazim = 0.0, delev = 0.0;
    double rot1 = 0.0, rot2 = 0.0, rot3 = 0.0;  //in radians
    double rot4 = 0.0, rot5 = 0.0, rot6 = 0.0;
    double rot12 = 0.0, rot22 = 0.0, rot32 = 0.0;  //in radians
    double rot42 = 0.0, rot52 = 0.0, rot62 = 0.0;
    int visualizeRobots = 1, visualize = 1;
    //double offsetX=1092.1, offsetY=3.4, offsetZ=0.1, offsetRot=0.064270;
    double offsetX=ROBOTS_DISTANCE, offsetY=0, offsetZ=0, offsetRot=0;

    bool withObs = true;

public:
	ABB_simulator();
	void spin(int frequency = 100);
protected:
	ros::NodeHandle node_handle_;
    void initialize();
	

}; 
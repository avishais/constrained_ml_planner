
#include "simulator.hpp"
#include "std_msgs/Float64MultiArray.h"
// #include "common_msgs_gl/SendDoubleArray.h"
// #include "common_msgs_gl/GetDoubleArray.h"
// #include "common_msgs_gl/SendIntArray.h"


ABB_simulator::ABB_simulator() : node_handle_("~"){
	initialize();
}

void ABB_simulator::initialize(){
}




// void MoveGripper::callbackGripperPose(std_msgs::Float64MultiArray msg) {
//     // pos_ref_.resize(msg.data.size());
// 	// for(size_t i = 0; i < msg.data.size(); i++){
// 	// 	pos_ref_[i] = msg.data[i];
// 	// }

//     // pub_pos_ref_.publish(msg);
// }

// void MoveGripper::callbackGripperVel(std_msgs::Float64MultiArray msg) {
//     vel_ref_.resize(msg.data.size());
// 	for(size_t i = 0; i < msg.data.size(); i++){
// 		vel_ref_[i] = msg.data[i];
// 	}

// 	// pub_vel_ref_.publish(msg);
// }




void ABB_simulator::spin(int frequency) {
    ros::Rate loop_rate(frequency);
    while(ros::ok()){
		// if(g.velMode()){
		// 	g.velocityModeStep();	
		// }
        // Move();
		ros::spinOnce();
		loop_rate.sleep();
    }
}




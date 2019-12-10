
#include "projector.h"
#include <chrono>
// #include "std_msgs/Float64MultiArray.h"
// #include "common_msgs_gl/SendDoubleArray.h"
// #include "common_msgs_gl/GetDoubleArray.h"
// #include "common_msgs_gl/SendIntArray.h"


ABB_projector::ABB_projector() : node_handle_("~"), kdl(ROBOTS_DISTANCE, ROD_LENGTH) 
{
	srv_project = node_handle_.advertiseService("/abb/project", &ABB_projector::callbackProject, this);
    srv_sample = node_handle_.advertiseService("/abb/sample", &ABB_projector::callbackSample, this);
}



bool ABB_projector::callbackProject(abb_projector::project_srv::Request& req, abb_projector::project_srv::Response& res) {
    State q = req.input;
    // printVector(q);
    auto start = chrono::steady_clock::now();
    
    if (!GD(q)) {
        return false;
    }

    auto end = chrono::steady_clock::now();
    res.time = chrono::duration_cast<chrono::microseconds>(end - start).count();
    res.output = get_GD_result();

    return true;
}

bool ABB_projector::callbackSample(abb_projector::sample_srv::Request& req, abb_projector::sample_srv::Response& res) {
    State q = sample_q();
    // for (int j = 0; j < q.size(); j++)
    //     cout << q[j] << " ";
    res.output = q;

    return true;
}

// void MoveGripper::callbackGripperVel(std_msgs::Float64MultiArray msg) {
//     vel_ref_.resize(msg.data.size());
// 	for(size_t i = 0; i < msg.data.size(); i++){
// 		vel_ref_[i] = msg.data[i];
// 	}

// 	// pub_vel_ref_.publish(msg);
// }

void ABB_projector::spin(int frequency) {
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




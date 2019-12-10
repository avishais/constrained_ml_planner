#include "kdl_class_abb.h"
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <math.h>
#include "ros/ros.h"
#include "autoencoder_projector/autoencoder.h"


double normDistance(State a1, State a2) {
	double sum = 0;
	for (int i=0; i < a1.size(); i++)
		sum += pow(a1[i]-a2[i], 2);
	return sqrt(sum);
}


int main(int argc, char **argv) {
    kdl K(900, 300);
    State q_rand(12), q(12), q_new(12);

    ros::init(argc, argv, "add_two_ints_client");
    ros::NodeHandle n;
    ros::ServiceClient en_client = n.serviceClient<autoencoder_projector::autoencoder>("/autoencoder/encode");
    ros::ServiceClient de_client = n.serviceClient<autoencoder_projector::autoencoder>("/autoencoder/decode");
    ros::init(argc, argv, "add_two_ints_client");

    std::fstream fs, ff;
    fs.open("test_kdl.txt", std::fstream::out);
    ff.open("test_ae.txt", std::fstream::out);

    State z_min = {-1.8882294, -1.2310271, -1.2174333, -1.2481797, -2.2183428, -1.9920528};
    State z_max = {1.2014925, 1.5075847, 1.4499509, 1.9526815, 2.4434128, 1.677999};
    State z_rand(6), z(6);
    autoencoder_projector::autoencoder ae;

    float rq = 1.0, rz = 0.3;
    float d = 0, t = 0;
    int j = 0;
    while (j < 1000) {
        cout << j << endl;
        for (int i = 0; i < q_rand.size(); i++)
            q_rand[i] = -PI + (double)rand()/RAND_MAX * 2*PI;
        if (!K.GD(q_rand)) 
            continue;
        q = K.get_GD_result();

        // KDL
        for (int i = 0; i < q_rand.size(); i++)
            q_rand[i] = -PI + (double)rand()/RAND_MAX * 2*PI;
        d = normDistance(q, q_rand);
        for (int i = 0; i < q_rand.size(); i++)
            q_rand[i] = q[i] + rq/d * (q_rand[i] - q[i]);

        auto start = chrono::steady_clock::now();
        if (!K.GD(q_rand)) 
            continue;
        q_new = K.get_GD_result();
        auto end = chrono::steady_clock::now();

        d = normDistance(q_rand, q_new)        ;
        t = chrono::duration_cast<chrono::microseconds>(end - start).count();
        fs << t << " " << d << endl;

        // AE
        ae.request.input.resize(12);
        
        for(int k = 0; k < 12; k++)
            ae.request.input[k] = q[k];
        if (en_client.call(ae)) {
            for(int k = 0; k < 6; k++)
                z[k] = ae.response.output[k];
        }
        else
            continue;

        for (int i = 0; i < z_rand.size(); i++)
            z_rand[i] = (double)rand()/RAND_MAX * (z_max[i]-z_min[i]) + z_min[i];
        d = normDistance(z, z_rand);
        for (int i = 0; i < z_rand.size(); i++)
            z_rand[i] = z[i] + rz/d * (z_rand[i] - z[i]);
        
        ae.request.input.resize(6);
        for(int k = 0; k < 6; k++)
            ae.request.input[k] = z_rand[k];
        if (de_client.call(ae)) {
            for(int k = 0; k < 12; k++)
                q_rand[k] = ae.response.output[k];
        }
        else
            continue;
        start = chrono::steady_clock::now();
        if (!K.GD(q_rand)) 
            continue;
        q_new = K.get_GD_result();
        end = chrono::steady_clock::now();

        d = normDistance(q_rand, q_new);
        t = chrono::duration_cast<chrono::microseconds>(end - start).count();
        ff << t << " " << d << endl;

        j++;
    }
    fs.close();
    ff.close();

}

/*
    std::fstream fs;
    fs.open("test_kdl.txt", std::fstream::in | std::fstream::out);

    int j = 0;
    float t = 0, d = 0;
    while (j < 10000) {
        cout << j << endl;
        for (int i = 0; i < q_rand.size(); i++)
            q_rand[i] = -PI + (double)rand()/RAND_MAX * 2*PI;

        auto start = chrono::steady_clock::now();
        if (!K.GD(q_rand)) {
            continue;
        }

        q = K.get_GD_result();
        auto end = chrono::steady_clock::now();
        j++;

        d = normDistance(q_rand, q);
        t = chrono::duration_cast<chrono::microseconds>(end - start).count();

        fs << t << " " << d << endl;
    }
    fs.close();

    State z_min = {-1.8882294, -1.2310271, -1.2174333, -1.2481797, -2.2183428, -1.9920528};
    State z_max = {1.2014925, 1.5075847, 1.4499509, 1.9526815, 2.4434128, 1.677999};
    State z_rand(12);
    autoencoder_projector::autoencoder ae;

    fs.open("test_ae.txt", std::fstream::in | std::fstream::out);
    j = 0;
    float t2 = 0, t1 = 0; 
    d = 0;
    while (j < 10000) {
        cout << j << endl;
        for (int i = 0; i < z_rand.size(); i++)
            z_rand[i] = (double)rand()/RAND_MAX * (z_max[i]-z_min[i]) + z_min[i];

        ae.request.input.resize(6);
        auto start = chrono::steady_clock::now();
        for(int k = 0; k < 6; k++)
            ae.request.input[k] = z_rand[k];
        if (de_client.call(ae)) {
            for(int k = 0; k < 12; k++)
                q_rand[k] = ae.response.output[k];
            t1 = ae.response.time;
        }
        else
            continue;
        auto end = chrono::steady_clock::now();
        // t1 = chrono::duration_cast<chrono::microseconds>(end - start).count();

        start = chrono::steady_clock::now();
        if (!K.GD(q_rand)) {
            continue;
        }
        q = K.get_GD_result();
        end = chrono::steady_clock::now();
        j++;

        d = normDistance(q_rand, q);
        t2 = chrono::duration_cast<chrono::microseconds>(end - start).count();

        fs << t1 << " " << t2 << " " << d << endl;
    }
    fs.close();

}*/
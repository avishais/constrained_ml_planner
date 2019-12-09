#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int16
from std_srvs.srv import SetBool, Empty, EmptyResponse
from autoencoder_projector.srv import autoencoder
from abb_projector.srv import sample_srv
import math
import numpy as np
from Autoencoder import Autoencoder

np.random.seed(10)

class Spin_ae_proj(Autoencoder):

    def __init__(self):
        Autoencoder.__init__(self)

        rospy.Service('/autoencoder/encode', autoencoder, self.encode)
        rospy.Service('/autoencoder/decode', autoencoder, self.decode)
        rospy.Service('/autoencoder/sample', sample_srv, self.sample_q)
        rospy.init_node('autoencoder_project', anonymous=True)

        print('[autoencoder] Ready.')

        rospy.spin()

    def encode(self, req):
        x = np.array(req.input)
        x = self.normalize(x)
        z = self.Encode(x)
        
        return {'output': z}

    def decode(self, req):
        z = np.array(req.input)
        x = self.Decode(z)
        x = self.denormalize(x)
        
        return {'output': x}

    def sample_q(self, req):
        z = self.sample_z()
        x = self.Decode(z)
        x = self.denormalize(x)
        
        return {'output': x}

        
if __name__ == '__main__':
    try:
        SP = Spin_ae_proj()
    except rospy.ROSInterruptException:
        pass
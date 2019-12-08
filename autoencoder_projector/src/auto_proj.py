#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int16
from std_srvs.srv import SetBool, Empty, EmptyResponse
from autoencoder_projector.srv import autoencoder
import math
import numpy as np
import autoencoder as ae

np.random.seed(10)

class Spin_ae_proj(ae):

    def __init__(self):
        ae.__init__(self)

        rospy.Service('/autoencoder/encode', autoencoder, self.encode)
        rospy.Service('/autoencoder/decode', autoencoder, self.decode)
        rospy.init_node('autoencoder_project', anonymous=True)

        self.load_model()
        print('[autoencoder] Ready.')

        rospy.spin()

    def load_model(self):
        pass



    def encode(self, req):
        X = np.array([0,0])

        
        return {'output': X}

    def decode(self, req):
        X = np.array([0,0])

        
        return {'output': X}


        
if __name__ == '__main__':
    try:
        SP = Spin_ae_proj()
    except rospy.ROSInterruptException:
        pass
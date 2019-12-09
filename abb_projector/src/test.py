#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int16
from std_srvs.srv import SetBool, Empty, EmptyResponse
from autoencoder_projector.srv import autoencoder
from abb_projector.srv import sample_srv, project_srv


encode_srv = rospy.ServiceProxy('/autoencoder/encode', autoencoder)
decode_srv = rospy.ServiceProxy('/autoencoder/decode', autoencoder)
smp_ae_srv = rospy.ServiceProxy('/autoencoder/sample', sample_srv)
proj_srv = rospy.ServiceProxy('/abb/project', project_srv)
smp_srv = rospy.ServiceProxy('/abb/sample', sample_srv)
rospy.init_node('test', anonymous=True)

F = open("/home/avishai/catkin_ws/src/ckc_ml_planner/abb_projector/data/samples.txt","w") 

N = 100

# F.write(str(N) + '\n') 
# for _ in range(100):
#     q = smp_srv().output
#     print q
#     for qq in q:
#         F.write(str(qq) + ' ') 
#     F.write('\n')
# F.close()

F.write(str(N) + '\n') 
for _ in range(100):
    q = smp_ae_srv().output
    print q
    q = proj_srv(q).output
    print q
    for qq in q:
        F.write(str(qq) + ' ') 
    F.write('\n')
F.close()

# F.write(str(10) + '\n') 
# x = smp_srv().output
# print x
# for _ in range(5):
#     for qq in x:
#         F.write(str(qq) + ' ') 
#     F.write('\n')
# z = encode_srv(x).output
# q = decode_srv(z).output
# print q
# for _ in range(5):
#     for qq in q:
#         F.write(str(qq) + ' ') 
#     F.write('\n')
# F.close()




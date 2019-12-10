#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Float32MultiArray, Int16
from std_srvs.srv import SetBool, Empty, EmptyResponse
from autoencoder_projector.srv import autoencoder
from abb_projector.srv import sample_srv, project_srv
import time
import numpy as np
import matplotlib.pyplot as plt


H1 = np.loadtxt('test_kdl.txt')
H1 = H1[H1[:,1].argsort()]
# H1 = H1[np.where(H1[:,0] < 1000.)]
print np.mean(H1[:,0]), np.mean(H1[:,1])

H2 = np.loadtxt('test_ae.txt')
H2 = H2[H2[:,1].argsort()]
# H2 = H2[np.where(H2[:,0] < 1000.)]
print np.mean(H2[:,0]), np.mean(H2[:,1])

plt.figure(1)
plt.plot(H2[:,1], H2[:,0]/1.e3, '.r')
plt.plot(H1[:,1], H1[:,0]/1.e3, '.b')

plt.figure(2)
plt.hist(H2[:,1], 20, color='red')
plt.hist(H1[:,1], 20, color='blue')

plt.show()

exit(1)


encode_srv = rospy.ServiceProxy('/autoencoder/encode', autoencoder)
decode_srv = rospy.ServiceProxy('/autoencoder/decode', autoencoder)
smp_ae_srv = rospy.ServiceProxy('/autoencoder/sample', sample_srv)
smp_z_srv = rospy.ServiceProxy('/autoencoder/sample_z', sample_srv)
proj_srv = rospy.ServiceProxy('/abb/project', project_srv)
smp_srv = rospy.ServiceProxy('/abb/sample', sample_srv)
rospy.init_node('test', anonymous=True)

F = open("/home/avishai/catkin_ws/src/ckc_ml_planner/abb_projector/data/samples_comau3.txt","w") 

N = 100

## Test regular sampling with kdl
# F.write(str(N) + '\n') 
# for _ in range(100):
#     q = smp_srv().output
#     print q
#     for qq in q:
#         F.write(str(qq) + ' ') 
#     F.write('\n')
# F.close()

## Test sampling in the Z space and then project
# F.write(str(N) + '\n') 
# for _ in range(100):
#     q = smp_ae_srv().output
#     q = proj_srv(q).output
#     for qq in q:
#         F.write(str(qq) + ' ') 
#     F.write('\n')
# F.close()

## Test random sample time
# M = 1000
# t1 = 0.
# t2 = 0.
# t3 = 0.
# for _ in range(M):
#     # kdl time
#     st1 = time.time()
#     q1 = smp_srv().output
#     t1 += time.time() - st1

#     # kdl+ae time
#     st2 = time.time()
#     q2 = smp_ae_srv().output
#     t3 += time.time() - st2
#     q2 = proj_srv(q2).output
#     t2 += time.time() - st2

# print 'kdl: ' + str(t1 / M) + ' sec'
# print 'ae: ' + str(t2 / M) + ' sec'
# print 'ae3: ' + str(t3 / M) + ' sec'

## Test propagation from a configuration on the manifold toward a random node
M = 1000
t1 = 0.
t1i = 0.
t2i = 0.
t2 = 0.
t3 = 0.
rq = 3.0
rz = 0.00
H1 = []
H2 = []
count = 0
F.write(str(2*M) + '\n') 
for _ in range(M):
    q = np.array(smp_srv().output)
    for qq in q:
        F.write(str(qq) + ' ') 
    F.write('\n')
    
    # kdl time
    st1 = time.time()
    q_rand = np.random.random(size=(18,)) * 2*np.pi - np.pi
    # d1 = np.linalg.norm(q - q_rand)
    # q1p_new = q + rq / d1 * (q_rand[0] - q)
    try:
        res = proj_srv(q_rand)
    except:
        continue
    q1_new = np.array(res.output)
    H1.append(np.array([time.time() - st1, res.time, np.linalg.norm(q - q_rand), np.linalg.norm(q_rand - q1_new)]))
    t1 += time.time() - st1
    t1i += res.time
    count += 1
    for qq in q1_new:
        F.write(str(qq) + ' ') 
    F.write('\n')
    # print(d1, np.linalg.norm(q - q1p_new), np.linalg.norm(q - q1_new))

    # kdl+ae time
    # st2 = time.time()
    # z = np.array(encode_srv(q).output)
    # z_rand = smp_z_srv().output
    # d2 = np.linalg.norm(z - z_rand)
    # z_new = z + rz / d2 * (z_rand - z)
    # q2p_new = np.array(decode_srv(z_new).output)
    # res = proj_srv(q2p_new)
    # q2_new = res.output
    # H2.append(np.array([time.time() - st2, np.linalg.norm(z - z_rand), np.linalg.norm(q2p_new - q2_new)]))
    # t2 += time.time() - st2
    # t2i += res.time
    
    # for qq in q2_new:
    #     F.write(str(qq) + ' ') 
    # F.write('\n')
    # print(d1, np.linalg.norm(q - q1p_new), np.linalg.norm(q - q1_new))
F.close()
print 'kdl: ' + str(t1 / count) + ' sec'
print 'kdl_internal: ' + str(t1i / count) + ' sec'

# print 'ae: ' + str(t2 / M) + ' sec'
# print 'ae_internal: ' + str(t2i / M) + ' sec'


H1 = np.array(H1)
# H2 = np.array(H2)
H1 = H1[H1[:,2].argsort()]
# H2 = H2[H2[:,2].argsort()]

plt.figure(1)
plt.plot(H1[:,2], H1[:,0], '.')

# plt.figure(2)
# plt.plot(H2[:,2], H2[:,0], '.')
plt.show()
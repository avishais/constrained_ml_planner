import matplotlib.pyplot as plt
import numpy as np

M = np.loadtxt('samples.txt', delimiter=' ', usecols=range(4))

plt.figure(1)
L = [0.5, 1, 0.7]
plt.plot(0, 0, 'ob')

for V in M[:100,:]:
    X = 0
    Y = 0
    p = 0
    for v, l in zip(V, L):
        p += v
        x = X + l*np.cos(p)
        y = Y + l*np.sin(p)
        plt.plot(x, y, 'or')
        plt.plot([X, x], [Y, y], '-k')

        X, Y = x, y

plt.axis('equal')


plt.figure(2)
plt.plot(M[:,0], M[:,1],'.')

plt.show()
    
    
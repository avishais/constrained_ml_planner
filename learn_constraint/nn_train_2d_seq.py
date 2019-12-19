""" 
Author: Avishai Sintov
"""
# from __future__ import division, print_function, absolute_import

# from nn_functions import * # My utility functions

import tensorflow as tf
tf.compat.v1.enable_eager_execution()

import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits import mplot3d


# ---------------------------------

Encoder = tf.keras.models.Sequential([
  tf.keras.layers.Dense(2, activation='relu'),
  tf.keras.layers.Dropout(0.1),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(10, activation='relu'),
  tf.keras.layers.Dropout(0.3),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(10, activation='relu'),
  tf.keras.layers.Dropout(0.3),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(2, activation='relu'),
  tf.keras.layers.Dropout(0.1),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(1, activation='relu'),
])

Decoder = tf.keras.models.Sequential([
  tf.keras.layers.Dense(10, activation='relu'),
  tf.keras.layers.Dropout(0.1),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(10, activation='relu'),
  tf.keras.layers.Dropout(0.3),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(10, activation='relu'),
  tf.keras.layers.Dropout(0.3),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(10, activation='relu'),
  tf.keras.layers.Dropout(0.1),
  tf.keras.layers.BatchNormalization(),
  tf.keras.layers.Dense(2, activation='sigmoid')
])

def loss(model, original):
  return tf.reduce_mean(tf.square(tf.subtract(model, original)))

def grad(model, inputs):
  with tf.GradientTape() as tape:
      reconstruction = model(inputs)
      loss_value = loss(inputs, reconstruction)
  return loss_value, tape.gradient(loss_value, model.trainable_variables), reconstruction

def next_batch(num, data):
    '''
    Return a total of `num` random samples. 
    Similar to mnist.train.next_batch(num)
    '''
    idx = np.arange(0 , len(data))
    np.random.shuffle(idx)
    idx = idx[:num]
    data_shuffle = [data[ i] for i in idx]

    return np.asarray(data_shuffle)

Input = tf.keras.layers.Input(shape=(2,))
Z = Encoder(Input)
Output = Decoder(Z)
Autoencoder = tf.keras.Model(inputs = Input, outputs = Output)
Autoencoder.compile(optimizer='adam', loss=loss, metrics=None)


# model_concat = tf.keras.layers.Concatenate([Encoder, Decoder])

# Autoencoder = tf.keras.Model(model_concat)

# Autoencoder.compile(optimizer='adam',
#               loss=loss,
#               metrics=None)


print('Loading training data...')

def func(x):
  return 2*x**2 + 5

X = []
P = np.random.random(100000)*100.0 - 50.0
for p in P:
  y = func(p)
  X.append([p, y])
X = np.array(X)
n = X.shape[1]

x_max = np.max(X, 0)
x_min = np.min(X, 0)
X = (X-x_min)/(x_max-x_min)


from sklearn.model_selection import train_test_split
X_train, X_test, Y_train, Y_test = train_test_split(X, X, test_size=0.05, random_state= 42)

# plt.plot(X_train[:,0],X_train[:,1],'.k')
# plt.plot(X_test[:,0],X_test[:,1],'.r')
# plt.show()



np.random.seed(1)
tf.random.set_seed(1)
epochs = 100
learning_rate = 1e-2
batch_size = 100

Autoencoder.fit(X_train, X_train, batch_size=batch_size, epochs=epochs, shuffle=True)

# opt = tf.optimizers.Adam(learning_rate=learning_rate)
# H = []
# global_step = tf.Variable(0)
# for epoch in range(epochs):
#     batch = next_batch(batch_size, X_train)
#     loss_value, grads, reconstruction = grad(Autoencoder, batch)
#     opt.apply_gradients(zip(grads, Autoencoder.trainable_variables))
        
#     if epoch % 10 == 0:
#       print("Step: {}, Loss: {}".format(epoch, loss(batch, reconstruction).numpy()))

#       x = Autoencoder(X_test[0].reshape(1,-1))
#       print(x.numpy(), X_test[0])

#     H.append([epoch, loss_value])
# H = np.array(H)
# plt.plot(H[:,0],H[:,1],'-')

fig = plt.figure()
plt.plot(X_test[:,0],X_test[:,1],'.k')
X_ae = Autoencoder(X_test)
plt.plot(X_ae[:,0],X_ae[:,1],'.r')

for _ in range(10):
  xt = np.random.random(2)*100.0 - 50.0
  xt[1] = func(xt[0])
  xt = (xt-x_min)/(x_max-x_min)

  plt.plot(xt[0],xt[1],'ob')
  x = Autoencoder(np.array([xt])).numpy()[0]
  plt.plot(x[0],x[1],'om')
  plt.plot([xt[0], x[0]],[xt[1], x[1]],'-y')

plt.show()




    
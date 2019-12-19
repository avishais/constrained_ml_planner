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
l2 = 0.01

class Encoder(tf.keras.Model):
  def __init__(self, intermediate_dim):
    super(Encoder, self).__init__()
    self.hidden_layer = tf.keras.layers.Dense(units=4, activation=tf.nn.relu, kernel_initializer='he_uniform', kernel_regularizer=tf.keras.regularizers.l2(l2))
    self.output_layer = tf.keras.layers.Dense(units=intermediate_dim, activation=tf.nn.relu, kernel_regularizer=tf.keras.regularizers.l2(l2))
    self.bn1 = tf.keras.layers.BatchNormalization()
    self.bn2 = tf.keras.layers.BatchNormalization()
    self.bn3 = tf.keras.layers.BatchNormalization()
    self.bn4 = tf.keras.layers.BatchNormalization()
    self.bn5 = tf.keras.layers.BatchNormalization()
    self.do1 = tf.keras.layers.Dropout(0.1)
    self.do2 = tf.keras.layers.Dropout(0.1)
    self.do3 = tf.keras.layers.Dropout(0.1)
    self.do4 = tf.keras.layers.Dropout(0.1)
    self.do5 = tf.keras.layers.Dropout(0.1)
    
  def call(self, x):
    x = self.hidden_layer(x)
    x = self.do1(x)
    x = self.bn1(x)
    x = self.hidden_layer(x)
    x = self.do2(x)
    x = self.bn2(x)
    x = self.hidden_layer(x)
    x = self.do3(x)
    x = self.bn3(x)
    x = self.hidden_layer(x)
    x = self.do4(x)
    x = self.bn4(x)
    x = self.hidden_layer(x)
    x = self.do5(x)
    x = self.bn5(x)
    return self.output_layer(x)

class Decoder(tf.keras.Model):
  def __init__(self, intermediate_dim, original_dim):
    super(Decoder, self).__init__()
    self.hidden_layer = tf.keras.layers.Dense(units=95, activation=tf.nn.relu, kernel_initializer='he_uniform', kernel_regularizer=tf.keras.regularizers.l2(l2))
    self.hidden_layer1 = tf.keras.layers.Dense(units=95, activation=tf.nn.relu, kernel_initializer='he_uniform', kernel_regularizer=tf.keras.regularizers.l2(l2))
    self.output_layer = tf.keras.layers.Dense(units=original_dim, activation=tf.nn.sigmoid, kernel_regularizer=tf.keras.regularizers.l2(l2))
    self.bn1 = tf.keras.layers.BatchNormalization()
    self.bn2 = tf.keras.layers.BatchNormalization()
    self.bn3 = tf.keras.layers.BatchNormalization()
    self.bn4 = tf.keras.layers.BatchNormalization()
    self.bn5 = tf.keras.layers.BatchNormalization()
    self.do1 = tf.keras.layers.Dropout(0.1)
    self.do2 = tf.keras.layers.Dropout(0.1)
    self.do3 = tf.keras.layers.Dropout(0.1)
    self.do4 = tf.keras.layers.Dropout(0.1)
    self.do5 = tf.keras.layers.Dropout(0.1)
  
  def call(self, x):
    x = self.hidden_layer(x)
    x = self.do1(x)
    x = self.bn1(x)
    x = self.hidden_layer1(x)
    x = self.do2(x)
    x = self.bn2(x)
    x = self.hidden_layer1(x)
    x = self.do3(x)
    x = self.bn3(x)
    x = self.hidden_layer1(x)
    x = self.do4(x)
    x = self.bn4(x)
    x = self.hidden_layer1(x)
    x = self.do5(x)
    x = self.bn5(x)
    return self.output_layer(x)

class Autoencoder(tf.keras.Model):
  def __init__(self, intermediate_dim, original_dim):
    super(Autoencoder, self).__init__()
    self.encoder = Encoder(intermediate_dim=intermediate_dim)
    self.decoder = Decoder(intermediate_dim=intermediate_dim, original_dim=original_dim)
  
  def call(self, input_features):
    code = self.encoder(input_features)
    reconstructed = self.decoder(code)
    return reconstructed

  def call_encoder(self, input_features):
    return self.encoder(input_features)

  def call_decoder(self, code):
    return self.decoder(code)

def loss(model, original):
  return tf.reduce_mean(tf.square(tf.subtract(model, original)))
  # return tf.losses.mean_squared_error(model, original)

# def train(loss, model, opt, original):
#   with tf.GradientTape() as tape:
#     gradients = tape.gradient(loss(model, original), model.trainable_variables)
#     gradient_variables = zip(gradients, model.trainable_variables)
#     opt.apply_gradients(gradient_variables)

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

# ---------------------------------

print('Loading training data...')

X = np.loadtxt('./model/samples.txt', delimiter = ' ', usecols=range(4))
X = np.array(X)
n = X.shape[1]

# from sklearn.preprocessing import StandardScaler
# scaler = StandardScaler()
# scaler.fit(X)
# X = scaler.transform(X)
# x_mean = scaler.mean_
# x_std = scaler.scale_
# X = X*x_std + x_mean # Denormalize or use scaler.inverse_transform(X)

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
batch_size = 100
epochs = 5000
learning_rate = 1e-2
intermediate_dim = 1
original_dim = n

autoencoder = Autoencoder(intermediate_dim=intermediate_dim, original_dim=original_dim)
opt = tf.optimizers.Adam(learning_rate=learning_rate)

checkpoint_path = "autoencoder_model.ckpt"

if 0:
  autoencoder.load_weights(checkpoint_path)
else:
  H = []
  global_step = tf.Variable(0)
  for epoch in range(epochs):
      batch = next_batch(batch_size, X_train)
      loss_value, grads, reconstruction = grad(autoencoder, batch)
      opt.apply_gradients(zip(grads, autoencoder.trainable_variables))
          
      if epoch % 10 == 0:
        print("Step: {}, Loss: {}".format(epoch, loss(batch, reconstruction).numpy()))

        x = autoencoder(X_test[0].reshape(1,-1))
        print(x.numpy(), X_test[0])

        H.append([epoch, loss_value])
      if loss(batch, reconstruction).numpy() < 1e-5:
        break
  autoencoder.save_weights(checkpoint_path)

  H = np.array(H)
  fig = plt.figure()
  plt.plot(H[:,0],H[:,1],'-')

# fig = plt.figure()
# ax = plt.axes(projection='3d')
# ax.scatter3D(X_test[:,0], X_test[:,1], X_test[:,3], color='yellow', s=5)
# X_ae = autoencoder(X_test)
# ax.scatter3D(X_ae[:,0], X_ae[:,1], X_ae[:,3], color='red', s=5)
# ax.view_init(elev=27., azim=12)

# Xt = next_batch(10, X_test)
# for xt in Xt:
#   ax.scatter3D(xt[0], xt[1], xt[3]+0.04,color='blue', s=50)
#   x = autoencoder(np.array([xt])).numpy()[0]
#   ax.scatter3D(x[0],x[1],x[3]+0.04,color='magenta',s=50)
#   ax.plot3D([xt[0], x[0]],[xt[1], x[1]],[xt[3], x[3]],color='k', linewidth=2)

fig = plt.figure()
plt.plot(X_test[:,0], X_test[:,3], '.y')
X_ae = autoencoder(X_test)
plt.plot(X_ae[:,0], X_ae[:,3], '.r')

Xt = next_batch(10, X_test)
for xt in Xt:
  plt.plot(xt[0], xt[3],'ob')
  x = autoencoder(np.array([xt])).numpy()[0]
  plt.plot(x[0], x[3],'om')
  plt.plot([xt[0], x[0]], [xt[3], x[3]], '-k')

# print(X_test.shape)
# Z = autoencoder.call_encoder(X_test)
# z_min, z_max = np.min(Z), np.max(Z)
# z = np.random.random(1)*(z_max-z_min) + z_min
# print("z: ", z_min, z_max, z)
# y = autoencoder.call_decoder(np.array([z])).numpy()[0]
# print(y)
# plt.plot(y[0],y[1],'<g')

# fig = plt.figure()
# plt.plot(np.sort(Z,0),'.')


plt.show()




    
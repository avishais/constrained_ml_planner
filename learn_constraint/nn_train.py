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

class Encoder(tf.keras.layers.Layer):
  def __init__(self, intermediate_dim):
    super(Encoder, self).__init__()
    self.hidden_layer = tf.keras.layers.Dense(
      units=intermediate_dim,
      activation=tf.nn.relu,
      kernel_initializer='he_uniform'
    )
    self.output_layer = tf.keras.layers.Dense(
      units=intermediate_dim,
      activation=tf.nn.sigmoid
    )
    
  def call(self, input_features):
    x = input_features#self.hidden_layer(input_features)
    return self.output_layer(x)

class Decoder(tf.keras.layers.Layer):
  def __init__(self, intermediate_dim, original_dim):
    super(Decoder, self).__init__()
    self.hidden_layer = tf.keras.layers.Dense(
      units=intermediate_dim,
      activation=tf.nn.relu,
      kernel_initializer='he_uniform'
    )
    self.output_layer = tf.keras.layers.Dense(
      units=original_dim,
      activation=tf.nn.sigmoid
    )
  
  def call(self, code):
    x = self.hidden_layer(code)
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

def loss(model, original):
  reconstruction_error = tf.reduce_mean(tf.square(tf.subtract(model, original)))
  return reconstruction_error
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

from sklearn.preprocessing import StandardScaler
scaler = StandardScaler()
scaler.fit(X)
X = scaler.transform(X)
x_mean = scaler.mean_
x_std = scaler.scale_
# X = X*x_std + x_mean # Denormalize or use scaler.inverse_transform(X)

from sklearn.model_selection import train_test_split
X_train, X_test, Y_train, Y_test = train_test_split(X, X, test_size=0.05, random_state= 42)

# Define the number of nodes
# n_inputs = n
# n_nodes_hl1 = 3
# n_nodes_hl2 = 1
# n_nodes_hl2 = n_nodes_hl1
# n_outputs = n_inputs

np.random.seed(1)
tf.random.set_seed(1)
batch_size = 100
epochs = 1000
learning_rate = 1e-1
intermediate_dim = 2
original_dim = n

autoencoder = Autoencoder(intermediate_dim=intermediate_dim, original_dim=original_dim)
opt = tf.optimizers.Adam(learning_rate=learning_rate)

# training_dataset = tf.data.Dataset.from_tensor_slices(X_train)
# training_dataset = training_dataset.batch(batch_size)
# training_dataset = training_dataset.shuffle(X.shape[0])
# training_dataset = training_dataset.prefetch(batch_size * 4)


# writer = tf.summary.create_file_writer('tmp')

# with writer.as_default():
#   with tf.summary.record_if(True):
#     for epoch in range(epochs):
#       for step, batch_features in enumerate(training_dataset):
#         train(loss, autoencoder, opt, batch_features)
#         loss_values = loss(autoencoder, batch_features)
#         original = tf.reshape(batch_features, (batch_features.shape[0], 4))
#         reconstructed = tf.reshape(autoencoder(tf.constant(batch_features)), (batch_features.shape[0], 4))
#         tf.summary.scalar('loss', loss_values, step=step)
#         # tf.summary.image('original', original, max_outputs=10, step=step)
#         # tf.summary.image('reconstructed', reconstructed, max_outputs=10, step=step)


H = []
global_step = tf.Variable(0)
for epoch in range(epochs):
    # print("Epoch: ", epoch)
    # for x in range(0, X_train.shape[0], batch_size):
    #     x_inp = X_train[x : x + batch_size]
    #     loss_value, grads, reconstruction = grad(autoencoder, x_inp)
    #     opt.apply_gradients(zip(grads, autoencoder.trainable_variables))

    batch = next_batch(batch_size, X_train)
    loss_value, grads, reconstruction = grad(autoencoder, batch)
    opt.apply_gradients(zip(grads, autoencoder.trainable_variables))
        
    if epoch % 10 == 0:
      print("Step: {}, Loss: {}".format(epoch, loss(batch, reconstruction).numpy()))

      x = autoencoder(X_test[0].reshape(1,-1))
      print(x.numpy(), X_test[0])

    H.append([epoch, loss_value])

H = np.array(H)
plt.plot(H[:,0],H[:,1],'-')


fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(X_test[:,0], X_test[:,1], X_test[:,3], 'gray')
X_ae = autoencoder(X_test)
ax.scatter3D(X_ae[:,0], X_ae[:,1], X_ae[:,3], 'red')

plt.show()




    
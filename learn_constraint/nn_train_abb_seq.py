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
import pickle

np.random.seed(1)
tf.random.set_seed(1)

# ---------------------------------
l2 = 0.0001

print('Loading training data...')

X = np.loadtxt('./model_ABB/samples.txt', delimiter = ' ', usecols=range(12))
X = np.array(X)
n = X.shape[1]
# intermediate_dim = 8


# H = [n, 9, 5, 3, 5, 10, 8, 7, intermediate_dim]

def train(X, H):
  Encoder = tf.keras.models.Sequential()
  for i in range(1,len(H)):
    Encoder.add(tf.keras.layers.Dense(H[i], input_dim=H[i-1], activation='relu'))#, kernel_regularizer=tf.keras.regularizers.l2(l2)))
    # if np.random.random() > 0.5:
    # Encoder.add(tf.keras.layers.Dropout(0.01))
    Encoder.add(tf.keras.layers.BatchNormalization())

  Decoder = tf.keras.models.Sequential()
  for i in range(len(H)-2,0,-1):
    Decoder.add(tf.keras.layers.Dense(H[i], input_dim=H[i+1], activation='relu'))#, kernel_regularizer=tf.keras.regularizers.l2(l2)))
    # if np.random.random() > 0.5:
    # Decoder.add(tf.keras.layers.Dropout(0.01))
    Decoder.add(tf.keras.layers.BatchNormalization())

  Decoder.add(tf.keras.layers.Dense(H[0], input_dim=H[1], activation='relu'))#, kernel_regularizer=tf.keras.regularizers.l2(l2)))


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
  # X = next_batch(200000,X)

  with open('x_bounds.obj', 'wb') as f:
      pickle.dump([x_min, x_max], f)

  from sklearn.model_selection import train_test_split
  X_train, X_test, Y_train, Y_test = train_test_split(X, X, test_size=0.05, random_state= 42)

  batch_size = 100
  epochs = 100
  # learning_rate = 1e-2
  original_dim = n

  Input = tf.keras.layers.Input(shape=(original_dim,))
  Z = Encoder(Input)
  Output = Decoder(Z)
  Autoencoder = tf.keras.Model(inputs = Input, outputs = Output)
  Autoencoder.compile(optimizer='adam', loss=loss, metrics=None)
  checkpoint_path = "autoencoder_model.ckpt"
  if 1:
    Autoencoder.load_weights(checkpoint_path)
  else:
    # Autoencoder.load_weights(checkpoint_path)
    Autoencoder.fit(X_train, X_train, batch_size=batch_size, epochs=epochs, shuffle=True, validation_data=(X_test, X_test))
    Autoencoder.save_weights(checkpoint_path)

  Xe = []
  for x, i in zip(X_test, range(1000)):
    x_e = Autoencoder(x.reshape(1,-1))
    # print(x_e.numpy(), x, np.linalg.norm(x_e.numpy()-x))
    x_e = x_e.numpy()*(x_max-x_min) + x_min
    Xe.append(x_e)
  Xe = np.array(Xe).reshape(-1,12)
  np.savetxt('./model_ABB/samples_ae.txt', Xe)

  # print('\nhistory dict:', Autoencoder.history)
  # H = Autoencoder.history['loss']

  # Evaluate the model on the test data using `evaluate`
  # print('\n# Evaluate on test data')
  results = Autoencoder.evaluate(X_test, X_test, batch_size=batch_size)
  # print('test loss, test acc:', results)
  return results, Autoencoder, Encoder, Decoder, X_test



d = 6
F = open("results_dim.txt","a") 

F.write(str(d) + ': ') 
H = [12, 200, 200]
H.append(d)

print('Training NN with dim ' + str(d) + '...')
r, Autoencoder, encoder, decoder, X_test = train(X, H)
F.write(str(r) + '\n')
print(H, r)
F.close()


x = X_test[100,:]
print(x, Autoencoder(np.array([x])))

Input = tf.keras.layers.Input(shape=(12,))
Z = encoder(Input)
Encoder = tf.keras.Model(inputs = Input, outputs = Z)
z = Encoder(np.array([x]))
print(z)

Z_Input = tf.keras.layers.Input(shape=(6,))
output = decoder(Z_Input)
Decoder = tf.keras.Model(inputs = Z_Input, outputs = output)
print(Decoder(z))

# Z = Encoder(X_test)
# z_max = np.max(Z, 0)
# z_min = np.min(Z, 0)
# print(z_min, z_max)
# with open('z_bounds.obj', 'wb') as f:
#   pickle.dump([z_min, z_max], f)


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


# plt.show()




    
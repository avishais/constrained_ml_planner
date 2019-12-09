#!/usr/bin/env python

import tensorflow as tf
import math
import numpy as np
import pickle

class Autoencoder(object):
    H = [12, 200, 200, 6]
    model = '/home/avishai/catkin_ws/src/ckc_ml_planner/autoencoder_projector/model/'

    def __init__(self):

        Encoder = tf.keras.models.Sequential()
        for i in range(1,len(self.H)):
            Encoder.add(tf.keras.layers.Dense(self.H[i], input_dim=self.H[i-1], activation='relu'))#, kernel_regularizer=tf.keras.regularizers.l2(l2)))
            Encoder.add(tf.keras.layers.BatchNormalization())

        Decoder = tf.keras.models.Sequential()
        for i in range(len(self.H)-2,0,-1):
            Decoder.add(tf.keras.layers.Dense(self.H[i], input_dim=self.H[i+1], activation='relu'))#, kernel_regularizer=tf.keras.regularizers.l2(l2)))
            Decoder.add(tf.keras.layers.BatchNormalization())
        Decoder.add(tf.keras.layers.Dense(self.H[0], input_dim=self.H[1], activation='relu'))#, kernel_regularizer=tf.keras.regularizers.l2(l2)))

        Input = tf.keras.layers.Input(shape=(self.H[0],))
        Z = Encoder(Input)
        Output = Decoder(Z)
        self.Autoencoder = tf.keras.Model(inputs = Input, outputs = Output)
        self.Autoencoder.load_weights(self.model + 'autoencoder_model.ckpt')

        x_input = tf.keras.layers.Input(shape=(self.H[0],))
        Z = Encoder(x_input)
        self.encoder_obj = tf.keras.Model(inputs = x_input, outputs = Z)
        
        z_input = tf.keras.layers.Input(shape=(self.H[-1],))
        X = Decoder(z_input)
        self.decoder_obj = tf.keras.Model(inputs = z_input, outputs = X)

        with open(self.model + 'x_bounds.obj', 'rb') as f:
            self.x_min, self.x_max = pickle.load(f)

        with open(self.model + 'z_bounds.obj', 'rb') as f:
            self.z_min, self.z_max = pickle.load(f)

    def normalize(self, X):
        return (X-self.x_min)/(self.x_max-self.x_min)

    def denormalize(self, X):
        return X*(self.x_max-self.x_min) + self.x_min

    def Encode(self, x):
        z = self.encoder_obj(np.array([x]))
        return z.numpy()[0]

    def Decode(self, z):
        x = self.decoder_obj(z.reshape(1,-1))
        return x.numpy()[0]

    def sample_z(self):
        z = np.random.random(size=(1,6)) * (self.z_max - self.z_min) + self.z_min
        return z


# D = Autoencoder()

# x = np.array([0.34364245, 0.6669798 , 0.11252582, 0.66930337, 0.79562356,
#        0.4259124 , 0.12515756, 0.25166184, 0.41806601, 0.31983298,
#        0.1408644 , 0.99129579])    

# # xn = D.normalize(x)
# print x

# z = D.Encode(x)
# print z, z.shape

# xn = D.Decode(z)
# print(xn)

# print D.denormalize(xn)
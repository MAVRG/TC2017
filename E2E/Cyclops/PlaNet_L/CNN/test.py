#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tensorflow as tf
import numpy as np
import cv2
import random
import glob
import numpy as np

def make_dataset(filenames):
	train_image = []
	train_r = []
	train_label = []

	for filename in filenames:
		degree = ["0"]
		for deg in degree:
			image_file = glob.glob(filename+"/video0/degree"+deg+"/*.png")
			image_file.sort()
			r = np.loadtxt(filename+"/video0/degree"+deg+"/Twist.csv",delimiter=",")
			label = np.loadtxt(filename+"/video0/degree"+deg+"/Twist_long.csv",delimiter=",")

			for l in range(len(image_file)/5):
			  image = cv2.imread(image_file[l])
			  train_image.append(image)
			  train_r.append(r[l][1])
			  train_label.append(label[l])

	train_image = np.asarray(train_image)/255.0
	train_r = np.asarray(train_r).reshape((len(train_r),1))
	train_label = np.asarray(train_label)

	return train_image,train_r,train_label

def weight_variable(shape):
    # --- define weight
    initial = tf.truncated_normal(shape, stddev=0.1)
    return tf.Variable(initial)

def bias_variable(shape):
    # --- define bias
    initial = tf.constant(0.1, shape=shape)
    return tf.Variable(initial)

def conv1d(x, W):
    # --- define convolution
    return tf.nn.conv2d(x, W, strides=[1, 2, 2, 1], padding='VALID')

def conv2d(x, W):
    # --- define convolution
    return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='VALID')

def lrelu(x):
    x1 = tf.maximum(x,x/5.5)
    return x1


def CNN(image,label,keep_prob):
    with tf.name_scope('norm1') as scope:
        norm1=tf.nn.lrn(image,4,bias=1.0,alpha=0.001/9.0,beta=0.75)

    with tf.name_scope('conv1') as scope:
        W_conv1 = weight_variable([5, 5, 3, 24])
        b_conv1 = bias_variable([24])
        h_conv1 = tf.nn.relu(conv1d(norm1, W_conv1) + b_conv1)

    with tf.name_scope('conv2') as scope:
        W_conv2 = weight_variable([5, 5, 24, 36])
        b_conv2 = bias_variable([36])
        h_conv2 = tf.nn.relu(conv1d(h_conv1, W_conv2) + b_conv2)

    with tf.name_scope('conv3') as scope:
        W_conv3 = weight_variable([5, 5, 36, 48])
        b_conv3 = bias_variable([48])
        h_conv3 = tf.nn.relu(conv1d(h_conv2, W_conv3) + b_conv3)

    with tf.name_scope('conv4') as scope:
        W_conv4 = weight_variable([3, 3, 48, 64])
        b_conv4 = bias_variable([64])
        h_conv4 = tf.nn.relu(conv2d(h_conv3, W_conv4) + b_conv4)

    with tf.name_scope('conv5') as scope:
        W_conv5 = weight_variable([3, 3, 64, 64])
        b_conv5 = bias_variable([64])
        h_conv5 = tf.nn.relu(conv2d(h_conv4, W_conv5) + b_conv5)

    h_conv5_flatten = tf.reshape(h_conv5, [-1,1152])

    with tf.name_scope('fc1') as scope:
        W_fc1 = weight_variable([1152, 1164])
        W_fc1_label = weight_variable([2, 1164])
        b_fc1 = bias_variable([1164])
        h_fc1 = tf.nn.relu(tf.matmul(h_conv5_flatten, W_fc1) + tf.matmul(label, W_fc1_label) + b_fc1)

    with tf.name_scope('fc2') as scope:
        W_fc2 = weight_variable([1164,100])
        b_fc2 = bias_variable([100])
        h_fc2 = tf.nn.relu(tf.matmul(h_fc1, W_fc2) + b_fc2)

    with tf.name_scope('fc3') as scope:
        W_fc3 = weight_variable([100,50])
        b_fc3 = bias_variable([50])
        h_fc3 = tf.nn.relu(tf.matmul(h_fc2, W_fc3) + b_fc3)

    with tf.name_scope('fc4') as scope:
        W_fc4 = weight_variable([50,10])
        b_fc4 = bias_variable([10])
        h_fc4 = tf.nn.relu(tf.matmul(h_fc3, W_fc4) + b_fc4)

    h_fc4_drop = tf.nn.dropout(h_fc4, keep_prob)

    with tf.name_scope('fc5') as scope:
        W_fc5 = weight_variable([10,1])
        b_fc5 = bias_variable([1])
        y_conv = tf.matmul(h_fc4_drop, W_fc5) + b_fc5


    return y_conv,W_fc1


image1 = tf.placeholder(tf.float32, [None, 66, 200, 3])
label1 = tf.placeholder(tf.float32, [None, 2])
y_r = tf.placeholder(tf.float32, [None, 1])
keep_prob = tf.placeholder(tf.float32)

y_conv,W_fc1 = CNN(image1,label1,keep_prob)

loss = tf.sqrt(tf.reduce_mean(tf.square(y_conv-y_r))) + 0.05 * tf.nn.l2_loss(W_fc1)
#0.01で失敗7/26
#0.1で26モデル
#0.05で26-1もでる
train_step = tf.train.AdamOptimizer(1e-5).minimize(loss)


saver = tf.train.Saver(keep_checkpoint_every_n_hours = 1.0)

sess = tf.InteractiveSession()
sess.run(tf.global_variables_initializer())

saver.restore(sess, "model/20170803model.ckpt")

filenames = ["/media/kerberos/Samsung_T31/20170807traintrain"]
test_image,test_r,test_label = make_dataset(filenames)

f = open("result.txt","a")

itter = test_image.shape[0]
for i in range(itter):
       result= sess.run(y_conv, feed_dict={image1: test_image[i:i+1],label1:test_label[i:i+1], y_r: test_r[i:i+1],keep_prob:1.0})
       f.write(str(result[0][0]))
       f.write("\n")
f.close()



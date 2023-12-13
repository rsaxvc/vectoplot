#!/usr/bin/python
import csv
import matplotlib.pyplot as plt
import numpy as np

yaw=[]
pitch=[]
roll=[]
score=[]
with open('output.csv', newline='') as csvfile:
	r = csv.reader(csvfile, delimiter=',', quotechar='|')
	next(r)
	for row in r:
		pipi = 3.14159 * 2.0
		yaw.append(60*float(row[0])/pipi)
		pitch.append(60*float(row[1])/pipi)
		roll.append(60*float(row[2])/pipi)
		score.append(float(row[3]))
yaw = np.array(yaw, np.float32)
pitch = np.array(pitch, np.float32)
roll = np.array(roll, np.float32)
score = np.array(score, np.float32)


axes={"yaw":yaw,"pitch":pitch,"roll":roll,"score":score}

def plot2d(x,y):
	plt.xlabel(x)
	plt.ylabel(y)
	plt.scatter(axes[x], axes[y], marker="+")
	plt.show()

def plot3d(x, y, z):
	fig = plt.figure()
	ax = fig.add_subplot(projection='3d')
	ax.set_xlabel(x)
	ax.set_ylabel(y)
	ax.set_zlabel(z)
	ax.scatter(axes[x], axes[y], axes[z], marker="+")
	plt.show()


#plot2d('yaw','score')
#plot2d('pitch','score')
#plot2d('roll','score')
plot3d('yaw', 'pitch', 'score')
plot3d('pitch', 'roll', 'score')
plot3d('roll', 'yaw', 'score')

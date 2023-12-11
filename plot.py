#!/usr/bin/python
import csv
import matplotlib.pyplot as plt

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
		score.append(int(row[3]))

# plot
plt.plot(yaw, score)
plt.show()

plt.plot(pitch, score)
plt.show()

plt.plot(roll, score)
plt.show()

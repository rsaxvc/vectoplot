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
		yaw.append(float(row[0]))
		pitch.append(float(row[1]))
		roll.append(float(row[2]))
		score.append(int(row[3]))

# plot
plt.plot(yaw, score)
plt.show()

plt.plot(pitch, score)
plt.show()

plt.plot(roll, score)
plt.show()

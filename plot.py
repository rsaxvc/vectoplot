#!/usr/bin/python
import csv
import matplotlib.pyplot as plt

x=[]
y=[]
with open('output.csv', newline='') as csvfile:
	r = csv.reader(csvfile, delimiter=',', quotechar='|')
	next(r)
	for row in r:
		x.append(float(row[0]))
		y.append(int(row[1]))

# plot
#plt.plot(x, y, linewidth=2.0)
print(x)
print(y)
plt.plot(x, y)
plt.show()

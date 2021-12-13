# coding:utf-8

import matplotlib.pyplot as plt

import csv

opfilename = './op.csv'
chfilename = './ch.csv'


def readPoints(filename):
    x = []
    y = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for line in reader:
            x.append(float(line[0]))
            y.append(float(line[1]))
    return x, y


plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams['font.size'] = 13

opx, opy = readPoints(opfilename)
chx, chy = readPoints(chfilename)
chx.append(chx[0])
chy.append(chy[0])

plt.scatter(opx, opy, label='points')
plt.plot(chx, chy, linestyle='--', marker='X',
         mfc='g', mec='r', ms=9, c='r', label='convex hull')

plt.title('Convex Hull')
plt.grid(ls='--', alpha=0.5)
plt.xlabel('X(m)')
plt.ylabel('Y(m)')
plt.legend()

plt.show()

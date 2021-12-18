# coding:utf-8

import matplotlib.pyplot as plt

import csv

pfilename = './pf.csv'
vfilename = './vf.csv'


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

opx, opy = readPoints(pfilename)

plt.scatter(opx, opy, label='points', marker='X', c='r')

with open(vfilename, 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        for i in range(int(len(line)/2-1)):
            x1 = float(line[2*i+0])
            y1 = float(line[2*i+1])
            x2 = float(line[2*i+2])
            y2 = float(line[2*i+3])
            plt.plot([x1, x2], [y1, y2], c='g', linestyle=':')

plt.title('Voronois')
plt.grid(ls='--', alpha=0.5)
plt.xlabel('X(m)')
plt.ylabel('Y(m)')
plt.legend()

plt.show()

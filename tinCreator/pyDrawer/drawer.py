# coding:utf-8

import matplotlib.pyplot as plt

import csv

pfilename = './pf.csv'
tfilename = './tf.csv'


def readPoints(filename):
    x = []
    y = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for line in reader:
            x.append(float(line[0]))
            y.append(float(line[1]))
    return x, y


def readTris(filename):
    t1 = []
    t2 = []
    t3 = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for line in reader:
            t1.append([float(line[0]), float(line[1])])
            t2.append([float(line[2]), float(line[3])])
            t3.append([float(line[4]), float(line[5])])
    return t1, t2, t3


plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams['font.size'] = 13

opx, opy = readPoints(pfilename)
t1, t2, t3 = readTris(tfilename)


plt.scatter(opx, opy, label='points')

for i in range(len(t1)):
    plt.plot([t1[i][0], t2[i][0], t3[i][0], t1[i][0]],
             [t1[i][1], t2[i][1], t3[i][1], t1[i][1]], c='r', linestyle=':', marker='X',
             mfc='g', mec='r', ms=9)

for i in range(1):
    plt.plot([t1[i][0], t2[i][0], t3[i][0], t1[i][0]],
             [t1[i][1], t2[i][1], t3[i][1], t1[i][1]], c='r', linestyle=':', marker='X',
             mfc='g', mec='r', ms=9, label='init tins')

plt.title('Triangles From Convex Hull')
plt.grid(ls='--', alpha=0.5)
plt.xlabel('X(m)')
plt.ylabel('Y(m)')
plt.legend()

plt.show()

# coding:utf-8

import matplotlib.pyplot as plt
import csv
import numpy as np
from numpy.core.fromnumeric import size

filename = "./gd.csv"

iterCount = []
vals = []
x = np.linspace(-0.5, 1.5)

with open(filename, 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        iterCount.append(float(line[0]))
        vals.append(float(line[1]))

plt.xkcd()
plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 15

plt.plot(x, 0.5*(x ** 2), color='g', linestyle='--',
         mec='b', label=r'$f(x)=0.5{x^2}$')

plt.plot(iterCount, vals, 'x', c='r', markersize=8, label="Descent point")

for i in range(6):
    plt.annotate('',
                 xy=(iterCount[i+1], vals[i+1]), xycoords='data',
                 xytext=(iterCount[i], vals[i]), textcoords='data',
                 size=15, va="center", ha="center",
                 bbox=dict(boxstyle="round4", fc="w"),
                 arrowprops=dict(arrowstyle="-|>",
                                 connectionstyle="arc3,rad=0.4",
                                 fc="w"),
                 )

plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title("Gradient Descent Curve")


plt.show()

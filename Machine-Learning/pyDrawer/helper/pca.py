# coding:utf-8

from os import read
import matplotlib.pyplot as plt
import numpy as np
import csv


def readData(filename):
    with open(filename, 'r') as file:
        x = []
        y = []
        reader = csv.reader(file)
        for line in reader:
            x.append(float(line[0]))
            y.append(float(line[1]))
        return (x, y)


def pointer(fromx, fromy, tox, toy):
    plt.annotate('',
                 xy=(tox, toy), xycoords='data',
                 xytext=(fromx, fromy), textcoords='data',
                 size=15, va="center", ha="center",
                 bbox=dict(boxstyle="round4", fc="w"),
                 arrowprops=dict(arrowstyle="-|>",
                                 connectionstyle="arc3,rad=0.1",
                                 fc="w"),
                 )


pca_origin = "./pca_origin.csv"
pca_res = "./pca_res.csv"

x1, y1 = readData(pca_origin)
x2, y2 = readData(pca_res)

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 15

plt.scatter(x1, y1, label="origin sample set", c='r',
            marker='x', alpha=np.random.random(len(x1)))
plt.scatter(x2, y2, label="result sample set", c='g',
            marker='x', alpha=np.random.random(len(x2)))

pointer(-10, 0, 10, 0)
pointer(0, -5, 0, 5)
pointer(20, -2.5, 30, 10)
pointer(29, 0, 22, 5)

plt.title("PCA Translate")
plt.xlabel("feature 1")
plt.ylabel("feature 2")
plt.legend()

plt.grid(alpha=0.5, linestyle='--')

plt.show()

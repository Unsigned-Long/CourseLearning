# coding:utf-8

import matplotlib.ticker as ticker
import csv
from os import read
import matplotlib.pyplot as plt
import numpy as np


filename_1 = "./move2.txt"

x_1 = []
y_1 = []

with open(filename_1) as file:
    reader = csv.reader(file)
    for row in reader:
        x_1.append(float(row[0]))
        y_1.append(float(row[1]))


# alpha = np.linspace(0.05, 1.0, len(x))

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13


plt.plot(y_1, x_1, marker='+', c='g',
         linestyle='--', mec='r', label="Trace")

# # s1[6.98745,-5.31634,1.37415]
# plt.scatter(-5.31634, 6.98745, marker='^', s=150, label="s1[6.987,-5.316]")
# # s2[-8.01194,-0.3009,-1.29636]
# plt.scatter(-0.3009, -8.01194,  marker='^', s=150, label="s2[-8.012,-0.301]")
# # s3[2.33304,-1.54,1.36799]
# plt.scatter(-1.54, 2.33304,  marker='^', s=150, label="s3[2.333,-1.54]")

# [5.99:-15.62:1.35]
plt.scatter(-15.62, 5.99, marker='o', c='magenta', s=150, label="Sat1")
# [-5.696:-17.73:1.35]
plt.scatter(-17.73, -5.696, marker='o', c='burlywood', s=150, label="Sat2")
# [-4.95:14.644:1.35]
plt.scatter(14.644, -4.95,  marker='o', c='b', s=150, label="Sat3")
# [4.72:20.54:1.38]
plt.scatter(20.54, 4.72, marker='o', c='c', s=150, label="Sat4")

plt.xlabel("East(m)")
plt.ylabel("North(m)")
plt.title("move2[newtonLS]")

plt.legend()

plt.show()

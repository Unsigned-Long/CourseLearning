# coding:utf-8

import matplotlib.ticker as ticker
import csv
from os import read
import matplotlib.pyplot as plt
import numpy as np


taylorSeries = "./static_taylorSeries.txt"
linear = "./static_linear.txt"
newtonLS = "./static_newtonLS.txt"
ceres = "./static_ceres.txt"
sequential = "./static_sequential.txt"

taylorSeries_x = []
taylorSeries_y = []

newtonLS_x = []
newtonLS_y = []

ceres_x = []
ceres_y = []

linear_x = []
linear_y = []

sequential_x = []
sequential_y = []

with open(taylorSeries) as file:
    reader = csv.reader(file)
    for row in reader:
        taylorSeries_x.append(float(row[0]))
        taylorSeries_y.append(float(row[1]))

with open(sequential) as file:
    reader = csv.reader(file)
    for row in reader:
        sequential_x.append(float(row[0]))
        sequential_y.append(float(row[1]))

with open(linear) as file:
    reader = csv.reader(file)
    for row in reader:
        linear_x.append(float(row[0]))
        linear_y.append(float(row[1]))

with open(newtonLS) as file:
    reader = csv.reader(file)
    for row in reader:
        newtonLS_x.append(float(row[0]))
        newtonLS_y.append(float(row[1]))

with open(ceres) as file:
    reader = csv.reader(file)
    for row in reader:
        ceres_x.append(float(row[0]))
        ceres_y.append(float(row[1]))

taylorSeries_alpha = np.linspace(0.05, 1.0, len(taylorSeries_x))
newtonLS_alpha = np.linspace(0.05, 1.0, len(newtonLS_x))
ceres_alpha = np.linspace(0.05, 1.0, len(ceres_x))
linear_alpha = np.linspace(0.05, 1.0, len(linear_x))
sequential_alpha = np.linspace(0.05, 1.0, len(sequential_x))

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

# taylorSeries:[5.65213,4.66259,1.3575]

plt.scatter(taylorSeries_y[-1:], taylorSeries_x[-1:], marker='+',
            c='r', label="CAL_MODE_RANGE[taylorSeries]")

plt.scatter(taylorSeries_y, taylorSeries_x, marker='+',
            alpha=taylorSeries_alpha, c='r')


# ceres:[5.79409,4.65336,1.37265]

plt.scatter(ceres_y[-1:], ceres_x[-1:], marker='+',
            c='g', label="CAL_MODE_RANGE[ceres]")

plt.scatter(ceres_y, ceres_x, marker='+',
            alpha=ceres_alpha, c='g')


# newtonLS:[5.67184,4.70805,1.3575]

plt.scatter(newtonLS_y[-1:], newtonLS_x[-1:], marker='+',
            c='b', label="CAL_MODE_RANGE[newtonLS]")

plt.scatter(newtonLS_y, newtonLS_x, marker='+',
            alpha=newtonLS_alpha, c='b')

# linear[6.10839,4.69535,1.3575]

plt.scatter(linear_y[-1:], linear_x[-1:], marker='+',
            c='y', label="CAL_MODE_RANGE[linear]")

plt.scatter(linear_y, linear_x, marker='+',
            alpha=linear_alpha, c='y')

# sequential[5.91358,4.6747,1.37075]

# plt.scatter(sequential_x[-1:], sequential_y[-1:], marker='+',
#             c='c', label="CAL_MODE_RANGE[sequential]")

# plt.scatter(sequential_x, sequential_y, marker='+',
#             alpha=sequential_alpha, c='c')

# all

plt.scatter(4.66259, 5.65213, marker='o', c='r',
            s=150, label="CAL_MODE_ALL[taylorSeries]")
plt.scatter(4.65336, 5.79409,  marker='o', c='g',
            s=150, label="CAL_MODE_ALL[ceres]")
plt.scatter(4.70805, 5.67184,  marker='o', c='b',
            s=150, label="CAL_MODE_ALL[newtonLS]")
plt.scatter(4.69535, 6.10839, marker='o', c='y',
            s=150, label="CAL_MODE_ALL[linear]")
plt.scatter(4.6747, 5.91358, marker='o', c='c',
            s=150, label="CAL_MODE_ALL[sequential]")

# real pos

plt.scatter(4.6843, 5.7553, marker='o', c='k', s=150, label="REAL_POS")

# # base station
# # [5.99:-15.62:1.35]
# plt.scatter(5.99, -15.62, marker='o', c='magenta', s=150, label="Sat1")
# # [-5.696:-17.73:1.35]
# plt.scatter(-5.696, -17.73, marker='o', c='burlywood', s=150, label="Sat2")
# # [-4.95:14.644:1.35]
# plt.scatter(-4.95, 14.644, marker='o', c='b', s=150, label="Sat3")
# # [4.72:20.54:1.38]
# plt.scatter(4.72, 20.54, marker='o', c='c', s=150, label="Sat4")

plt.xlabel("East(m)")
plt.ylabel("North(m)")
plt.title("Range[10]-Step[5]")

plt.legend()

plt.show()

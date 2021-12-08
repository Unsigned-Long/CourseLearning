# coding:utf-8

import csv
import matplotlib.pyplot as plt

sample_x = []

sample_y = []

with open("./sample.csv", 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        sample_x.append(float(line[0]))
        sample_y.append(float(line[1]))

regress_x = []

regress_y = []

with open("./regression.csv", 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        regress_x.append(float(line[0]))
        regress_y.append(float(line[1]))

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

plt.scatter(sample_x, sample_y, marker='x', s=40, c='r', label="sample set")

plt.plot(regress_x, regress_y, color='g', linestyle='--', label="hypothesis")

plt.legend()

plt.title("Subsection Regression")
plt.xlabel("X")
plt.ylabel("Y")

plt.show()

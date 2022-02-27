# coding:utf-8

import math
from attr import s
from matplotlib.markers import MarkerStyle
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

time = []
val1 = []
val2 = []

with open("../data/signal.csv") as file:
    reader = csv.reader(file)
    for elem in reader:
        time.append(float(elem[0])/1000.0)
        val1.append(float(elem[1]))
        val2.append(float(elem[2]))

startpos = 1
endpos = len(val2)-1

fig, axs = plt.subplots(2, 1)

axs[0].plot(time[startpos:endpos], val1[startpos:endpos],
            linewidth=1.5, alpha=0.75, label="origin", linestyle="--", c='gray')
axs[0].grid(ls='--', alpha=0.5)
axs[0].legend()
axs[0].set_ylabel('Acceleration after HMA')

axs[1].plot(time[startpos:endpos], val2[startpos:endpos],
            linewidth=1.5, alpha=0.75, label="process", c='teal')
axs[1].legend()
for index in range(startpos, endpos):
    if val2[index-1] < val2[index] and val2[index] > val2[index+1]:
        axs[1].plot(time[index], val2[index],
                    linewidth=1.5, alpha=0.75, label="process", marker='o',
                    markersize=7, mfc='red', mec='darkred')

axs[1].set_ylabel('Signals')
axs[1].grid(ls='--', alpha=0.5)
axs[0].set_title('The Signals after HMA Process')
plt.xlabel('Time Stamp')
plt.show()

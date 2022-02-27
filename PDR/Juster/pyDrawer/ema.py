# coding:utf-8

from attr import s
from matplotlib.markers import MarkerStyle
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

time = []
yaw_origin = []

with open("../data/orientation.csv") as file:
    reader = csv.reader(file)
    for elem in reader:
        time.append(float(elem[0])/1000.0)
        yaw_origin.append(float(elem[1]))


def ema(data, param):
    result = []
    result.append(data[0])
    for i in range(1, len(data)):
        result.append(result[i-1]*param+(1.0-param)*data[i])
    return result


yaw_fir = ema(yaw_origin, 0.7)
yaw_sed = ema(yaw_fir, 0.8)
yaw_trd = ema(yaw_sed, 0.95)

startpos = 4000
endpos = 4500

fig, axs = plt.subplots(3, 1)

axs[0].plot(time[startpos:endpos], yaw_origin[startpos:endpos],
            linewidth=1.5, alpha=0.75, label="origin", linestyle="--", c='gray')
axs[0].grid(ls='--', alpha=0.5)
axs[0].legend()
axs[0].set_title('Orientation(Yaw) With EAM')

axs[1].plot(time[startpos:endpos], yaw_sed[startpos:endpos],
            linewidth=1.5, alpha=0.75, label="ema(0.7, 0.8)", linestyle="--", c='green')
axs[1].grid(ls='--', alpha=0.5)
axs[1].legend()

axs[2].plot(time[startpos:endpos], yaw_trd[startpos:endpos],
            linewidth=1.5, alpha=0.75, label="ema(0.7, 0.8, 0.9)", linestyle="--", c='darkred')
axs[2].grid(ls='--', alpha=0.5)
axs[2].legend()
axs[2].set_xlabel('Time Stamp')

plt.show()

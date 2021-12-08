# coding:utf-8

import matplotlib.pyplot as plt
import numpy as np
from numpy.lib.function_base import average
import reader
import math


def stdevp(mean: float, dataList: list) -> float:
    diff = 0.0
    for val in dataList:
        diff += pow(val-mean, 2)
    return math.sqrt(diff/len(dataList))


lat, long, alt = reader.read("../data/origin/altFile.txt")
v1, v2, originGA = reader.read("../data/origin/originalGAFile.txt")
v1, v2, spayialGA = reader.read("../data/origin/spatialGAFile.txt")
v1, v2, simBouGA = reader.read("../data/origin/simpleBouguerGAFile.txt")

originGA_min = min(originGA)
originGA_max = max(originGA)
originGA_avg = average(originGA)
or_min_index = originGA.index(originGA_min)
or_max_index = originGA.index(originGA_max)
or_min_lat = lat[or_min_index]
or_min_long = long[or_min_index]
or_max_lat = lat[or_max_index]
or_max_long = long[or_max_index]
print("min or pos", or_min_lat, or_min_long)
print("max or pos", or_max_lat, or_max_long)
print("min originGA", originGA_min)
print("max originGA", originGA_max)
print("mean originGA", originGA_avg)
print("stdevp originGA", stdevp(originGA_avg, originGA))
print()
spayialGA_min = min(spayialGA)
spayialGA_max = max(spayialGA)
spayialGA_avg = average(spayialGA)
sp_min_index = spayialGA.index(spayialGA_min)
sp_max_index = spayialGA.index(spayialGA_max)
sp_min_lat = lat[sp_min_index]
sp_min_long = long[sp_min_index]
sp_max_lat = lat[sp_max_index]
sp_max_long = long[sp_max_index]
print("min sp pos", sp_min_lat, sp_min_long)
print("max sp pos", sp_max_lat, sp_max_long)
print("min spayialGA", spayialGA_min)
print("max spayialGA", spayialGA_max)
print("mean spayialGA", spayialGA_avg)
print("stdevp spayialGA", stdevp(spayialGA_avg, spayialGA))
print()
simBouGA_min = min(simBouGA)
simBouGA_max = max(simBouGA)
simBouGA_avg = average(simBouGA)
sb_min_index = simBouGA.index(simBouGA_min)
sb_max_index = simBouGA.index(simBouGA_max)
sb_min_lat = lat[sb_min_index]
sb_min_long = long[sb_min_index]
sb_max_lat = lat[sb_max_index]
sb_max_long = long[sb_max_index]
print("min sb pos", sb_min_lat, sb_min_long)
print("max sb pos", sb_max_lat, sb_max_long)
print("min simBouGA", simBouGA_min)
print("max simBouGA", simBouGA_max)
print("mean simBouGA", simBouGA_avg)
print("stdevp simBouGA", stdevp(simBouGA_avg, simBouGA))

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 15

# Obver Sat
plt.scatter(long, lat, alpha=np.random.random(len(long))
            * 0.5+0.5, c='r', s=25, marker='x', label="ObverSat")

# origin GA
plt.scatter(or_min_long, or_min_lat, c='b',
            s=200, marker='v', label="originGA_min")
plt.scatter(or_max_long, or_max_lat, c='b',
            s=200, marker='^', label="originGA_max")

# spayialGA GA
plt.scatter(sp_min_long, sp_min_lat, c='g',
            s=200, marker='h', label="spayialGA_min")
plt.scatter(sp_max_long, sp_max_lat, c='g', s=200,
            marker='H', label="spayialGA_max")

# # simBouGA GA
plt.scatter(sb_min_long, sb_min_lat, c='black',
            s=400, marker='3', label="simBouGA_min")
plt.scatter(sb_max_long, sb_max_lat, c='black', s=400,
            marker='4', label="simBouGA_max")


plt.xlabel("long[Degree]")
plt.ylabel("lat[Degree]")
plt.grid(True, linestyle='--', linewidth=1, alpha=0.7)
plt.legend()
plt.title("Gravity Observation Position")

plt.show()

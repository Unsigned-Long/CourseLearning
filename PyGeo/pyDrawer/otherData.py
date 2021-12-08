# coding:utf-8

from matplotlib.cm import ScalarMappable
import matplotlib.pyplot as plt
import numpy as np
import reader

lat, long, alt = reader.read("../data/inter/spatialGAFile_inter.txt")

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 15

sc = plt.scatter(long, lat, alpha=np.random.random(len(long))
                 * 0.5+0.5, c=alt, marker='o')

plt.xlabel("long[Degree]")
plt.ylabel("lat[Degree]")


plt.title("Area Spatial GA")

plt.show()

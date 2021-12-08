# coding:utf-8
import matplotlib.pyplot as plt
import numpy as np

# 0.95,0.133334
# {1.0}, 1.0
# {2.0}, 2.2
# {3.0}, 2.9

x = [1.0, 2.0, 3.0]
y = [1.0, 2.2, 2.9]

x_pre = np.linspace(0.0, 4.0, 2)
y_pre = x_pre*0.95+0.133334

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

plt.scatter(x, y, marker='x', s=40, c='r', label="sample set")

plt.plot(x_pre, y_pre, color='g', linestyle='--', label="hypothesis")

plt.legend()

plt.title("Line Regression")
plt.xlabel("X")
plt.ylabel("Y")

plt.show()

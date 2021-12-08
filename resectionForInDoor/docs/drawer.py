# coding:utf-8

import matplotlib.pyplot as plt
import numpy as np

# estimate - real
# POS_6 [-7.187, -4.709, 0.379]
# POS_7 [0.850, -2.738, -0.488]
# POS_8 [-0.048, -4.5112, -1.181]

labels = ["X", "Y", "Z"]
pos6 = [-7.187, -4.709, 0.379]
pos7 = [0.850, -2.738, -0.488]
pos8 = [-0.048, -4.5112, -1.181]

plt.xkcd()
plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

x = np.arange(len(labels))  # the label locations
width = 0.25  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(x - width, pos6, width, label='POS_6', color='r', alpha=0.75)
rects2 = ax.bar(x, pos7, width, label='POS_7', color='g', alpha=0.75)
rects3 = ax.bar(x + width, pos8, width, label='POS_8', color='b', alpha=0.75)

ax.set_ylabel('Error(cm)')
ax.set_title('Point Position Error')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()

ax.bar_label(rects1, padding=3)
ax.bar_label(rects2, padding=3)
ax.bar_label(rects3, padding=3)

fig.tight_layout()

plt.show()

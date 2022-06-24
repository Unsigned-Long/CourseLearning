import math
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

lasLeftX = []
lasLeftY = []
lasRightX = []
lasRightY = []

staLeftX = []
staLeftY = []
staRightX = []
staRightY = []

with open("../../pyDrawer/section/pairs.csv") as file:
    lines = csv.reader(file)
    for line in lines:
        lasLeftX.append(float(line[0]))
        lasLeftY.append(float(line[1]))
        staLeftX.append(float(line[2]))
        staLeftY.append(float(line[3]))

        lasRightX.append(float(line[4]))
        lasRightY.append(float(line[5]))
        staRightX.append(float(line[6]))
        staRightY.append(float(line[7]))

fig, axs = plt.subplots(1, 2, sharey=True, figsize=(10, 7))


def mid(x1, y1, x2, y2):
    return [(x1+x2)/2, (y1+y2)/2]


def dis(x1, y1, x2, y2):
    return math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))


for i in range(len(lasLeftX)):
    axs[0].plot([lasLeftX[i], lasRightX[i]],
                [lasLeftY[i],  lasRightY[i]], ls='--', marker='X', c='green')

    lasMid = mid(lasLeftX[i], lasLeftY[i], lasRightX[i], lasRightY[i])
    lasDis = dis(lasLeftX[i], lasLeftY[i], lasRightX[i], lasRightY[i])
    axs[0].text(lasMid[0], lasMid[1], "{:.3f}".format(lasDis), size=12,
                ha="center", va="center",
                bbox=dict(boxstyle="round",
                          ec=(0.0, 1., 0.2),
                          fc=(0.8, 1., 0.8),
                          )
                )

    axs[1].plot([staLeftX[i], staRightX[i]],
                [staLeftY[i], staRightY[i]], ls='--', marker='X', c='red')
    staMid = mid(staLeftX[i], staLeftY[i], staRightX[i], staRightY[i])
    staDis = dis(staLeftX[i], staLeftY[i], staRightX[i], staRightY[i])
    axs[1].text(staMid[0], staMid[1], "{:.3f}".format(staDis), size=12,
                ha="center", va="center",
                bbox=dict(boxstyle="round",
                          ec=(1., 0.5, 0.5),
                          fc=(1., 0.8, 0.8),
                          )
                )

axs[0].grid(ls='--', alpha=0.5)
axs[0].set_xlabel('X(M)')
axs[0].set_ylabel('X(M)')
axs[0].set_title('Laser Point Data')

axs[1].grid(ls='--', alpha=0.5)
axs[1].set_xlabel('X(M)')
axs[1].set_ylabel('X(M)')
axs[1].set_title('Station Point Data')

fig.suptitle('Section')

plt.savefig("../../pyDrawer/section/img.png", dpi=100)
# plt.show()

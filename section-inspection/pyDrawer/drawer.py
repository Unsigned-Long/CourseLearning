from cProfile import label
import math
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13
plt.figure(figsize=(10, 10))

pts = [[3.04879, 3.10357], [3.38393, 2.53982], [3.21229, 2.95376],
       [3.54259, 2.23094], [3.29879, 2.89822], [3.50626, 2.24209],
       [2.86257, 3.18924], [3.17908, 2.90415], [3.01545, 3.08704],
       [3.33735, 2.52897], [3.24758, 2.76059], [3.17952, 2.99792],
       [3.3853, 2.64791], [3.4066, 2.64769], [3.5686, 2.05938],
       [3.47197, 2.55121], [3.34027, 2.51917], [3.31732, 2.77514],
       [2.82119, 3.16284], [3.28373, 2.79021], [3.00889, 3.15892],
       [3.26484, 2.92511], [3.45197, 2.56267], [3.13875, 3.09877],
       [3.32249, 2.88966], [2.98487, 3.24463], [3.29983, 2.83837],
       [3.23583, 2.77363], [3.27677, 2.85653], [3.51604, 2.47684]]

x = [pts[idx][0] for idx in range(len(pts))]
y = [pts[idx][1] for idx in range(len(pts))]


def drawCircle(name, cx, cy, r):
    xcir = []
    ycir = []
    theta = 0.0
    while theta < 2.0*math.pi:
        xcir.append(r*math.cos(theta)+cx)
        ycir.append(r*math.sin(theta)+cy)
        theta += 0.1
    xcir.append(r+cx)
    ycir.append(cy)
    plt.plot(xcir, ycir, "--", alpha=0.75, label=name)
    plt.scatter(cx, cy, marker='+')


drawCircle("ransac", 1.54825, 1.77176, 2.04072)
drawCircle("gauss-newton", 2.03951, 1.98116, 1.50319)
drawCircle("truth", 2, 2, 1.5)

plt.scatter(x, y, c='darkred', alpha=0.75, marker='X', s=70)
plt.title("Circle Fitting")
plt.xlabel('x')
plt.ylabel('y')
plt.grid(ls='--', alpha=0.5)
plt.legend()
plt.savefig("./img/circle_fit.png", dpi=500)
plt.show()

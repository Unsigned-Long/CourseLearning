from cProfile import label
import math
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13
plt.figure(figsize=(10, 10))

pts = [[0.668422, 0.0769819], [6.53919, 5.26929], [7.62198, 7.01191], [4.77732, 4.36411], [7.25412, 6.29543], [4.14293, 2.74588], [5.64899, 4.08767], [8.03073, 6.51254], [5.14659, 4.46023], [8.3642, 6.80562], [7.9477, 6.34717], [7.02989, 5.87989], [5.88119, 5.28548], [7.28608, 6.76237], [9.70087, 9.17848], [7.53863, 6.95679], [0.799169, 0.0425392], [7.0932, 6.26861], [3.7191, 2.80289], [2.97022, 1.60274], [5.19988, 4.77804], [1.88276, 1.22507], [4.24336, 2.59014], [8.36207, 6.87414], [1.60083, 1.23849], [
    4.59763, 3.82822], [3.78366, 2.61779], [3.45872, 1.91984], [2.21656, 1.4071], [5.02275, 4.41313], [1.07181, 0.350595], [6.11111, 5.51829], [9.52431, 8.58766], [4.24134, 3.80608], [6.46495, 4.79779], [7.94881, 6.65973], [3.20611, 2.88654], [3.57526, 2.93043], [8.37082, 7.19795], [6.71648, 6.23243], [1.14529, 0.182512], [2.27488, 1.41179], [5.17596, 4.67209], [1.92647, 0.312551], [4.08626, 2.74869], [9.99092, 9.61921], [6.0802, 5.13759], [5.71627, 5.23785], [8.44641, 6.78709], [5.55453, 4.02784]]

x = [pts[idx][0] for idx in range(len(pts))]
y = [pts[idx][1] for idx in range(len(pts))]


def drawLine(name, a, b, c):
    # ax + by + c = 0
    if abs(b) < 1E-10:
        xline = [-c/a, -c/a]
        yline = [0, 10]
    elif abs(a) < 1E-10:
        xline = [0, 10]
        yline = [-c/b, -c/b]
    else:
        xline = [0, 10]
        yline = [-c/b, (-c-10*a)/b]
    plt.plot(xline, yline, "--", label=name)


drawLine('truth', -1, 1, 1)
drawLine('ransca', -0.594865, 0.565093, 0.57167)
drawLine('fit', 0.594102, -0.600518, -0.535183)

plt.scatter(x, y, c='darkred', alpha=0.75,
            marker='X', s=70, label="data points")

plt.title("SLine Fitting [a:b:c]-[-1:1:1]")
plt.xlabel('x')
plt.ylabel('y')
plt.xlim((0, 10))
plt.ylim((0, 10))
plt.grid(ls='--', alpha=0.5)
plt.legend()
plt.savefig("./img/sline_fit_1.png", dpi=500)
plt.show()
from cProfile import label
import matplotlib.pyplot as plt
import csv

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(projection='3d')


def draw(filename, figname):
    x = []
    y = []
    z = []
    with open(filename) as file:
        lines = csv.reader(file)

        for line in lines:
            x.append(float(line[0]))
            y.append(float(line[1]))
            z.append(float(line[2]))
    ax.scatter(x, y, z, label=figname)


draw("./pyDrawer/pc1.csv", "Point Cloud1")
draw("./pyDrawer/pc2.csv", "Point Cloud1")
draw("./pyDrawer/result.csv", "From PC1 To PC2")

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.title("ICP")
plt.legend()
# plt.savefig("./img/icp.png", dpi=500)
plt.show()

import csv
import matplotlib.pyplot as plt

data = []
with open("./sample2d.csv", 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        data.append([float(line[0]), float(line[1]), int(line[2])])

dict = {0: {'x': [], 'y': []},
        1: {'x': [], 'y': []},
        2: {'x': [], 'y': []},
        3: {'x': [], 'y': []}}

for elem in data:
    dict[elem[2]]['x'].append(elem[0])
    dict[elem[2]]['y'].append(elem[1])

plt.xkcd()

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13


plt.scatter(dict[0]['x'], dict[0]['y'], marker='o',
            s=30, c='r', label="same", alpha=0.5)

plt.scatter(dict[1]['x'], dict[1]['y'], marker='o',
            s=30, c='g', label="different", alpha=1.0)

# plt.scatter(dict[2]['x'], dict[2]['y'], marker='o',
#             s=30, c='b', label="class 2", alpha=1.0)

# plt.scatter(dict[3]['x'], dict[3]['y'], marker='o',
#             s=30, c='c', label="class 3", alpha=1.0)

# centers
# 0,-6.65414,-1.58156
# 1, 2.14999,-5.38522
# 2, 0.110234,3.33687
# 3, -0.874248,-0.510288

# 0, {'values': [-11.439032, -0.750244], 'label': 0}
# 1, {'values': [4.644463, -9.793254], 'label': 1}
# 2, {'values': [2.069699, 5.801828], 'label': 2}
# 3, {'values': [-2.084194, -2.317153], 'label': 3}

# plt.scatter(-11.439032, -0.750244, marker='o',
#             s=130, c='r', label="center 0", edgecolors='k', linewidths=2)

# plt.scatter(4.644463, -9.793254, marker='o',
#             s=130, c='g', label="center 1", edgecolors='k', linewidths=2)

# plt.scatter(2.069699, 5.801828, marker='o',
#             s=130, c='b', label="center 2", edgecolors='k', linewidths=2)

# plt.scatter(-2.084194, -2.317153, marker='o',
#             s=130, c='c', label="center 3", edgecolors='k', linewidths=2)

plt.legend()

plt.title("Difference Between Min Distance And Navie Bayes ")
# plt.title("K-Means")
plt.xlabel("X")
plt.ylabel("Y")

plt.show()

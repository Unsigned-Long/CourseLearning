import csv
import matplotlib.pyplot as plt

data = []
with open("./sample3d.csv", 'r') as file:
    reader = csv.reader(file)
    for line in reader:
        data.append([float(line[0]), float(line[1]),
                    float(line[2]), int(line[3])])

dict = {0: {'x': [], 'y': [], 'z': []},
        1: {'x': [], 'y': [], 'z': []},
        2: {'x': [], 'y': [], 'z': []},
        3: {'x': [], 'y': [], 'z': []}}

for elem in data:
    dict[elem[3]]['x'].append(elem[0])
    dict[elem[3]]['y'].append(elem[1])
    dict[elem[3]]['z'].append(elem[2])

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

fig = plt.figure()
ax = fig.add_subplot(projection='3d')


ax.scatter(dict[0]['x'], dict[0]['y'], dict[0]
           ['z'], marker='o', c='r', alpha=0.5, label="class 0")
ax.scatter(dict[1]['x'], dict[1]['y'], dict[1]
           ['z'], marker='o', c='g', alpha=0.5, label="class 1")
ax.scatter(dict[2]['x'], dict[2]['y'], dict[2]
           ['z'], marker='o', c='b', alpha=0.5, label="class 2")
ax.scatter(dict[3]['x'], dict[3]['y'], dict[3]
           ['z'], marker='o', c='c', alpha=0.5, label="class 3")

# 0, -11.5583,-0.560278,3.13886
# 1, 5.10455,-1.14863,-12.4555
# 2, -4.24978,-15.2633,-1.46986
# 3,  1.10895,3.88889,-0.222528

# 0, -5.91215,-0.259816,1.72843
# 1, 1.96696,-4.03401,-6.14734
# 2,  -3.79157,-6.50849,0.00560777
# 3, 0.113951,2.04005,-3.9876
ax.scatter(-5.91215, -0.259816, 1.72843, marker='*',
           c='r', s=200, edgecolors='k', linewidths=1, label="center 0")
ax.scatter(1.96696, -4.03401, -6.14734, marker='*',
           c='g', s=200, edgecolors='k', linewidths=1, label="center 1")
ax.scatter(-3.79157, -6.50849, 0.00560777, marker='*',
           c='b', s=200, edgecolors='k', linewidths=1, label="center 2")
ax.scatter(0.113951, 2.04005, -3.9876, marker='*',
           c='c', s=200, edgecolors='k', linewidths=1, label="center 3")


ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# plt.title("Max Min Distance")
plt.title("K-Means")

ax.legend()

plt.show()

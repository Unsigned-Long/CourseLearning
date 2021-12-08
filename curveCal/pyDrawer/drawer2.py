# coding:utf-8

from os import read
import matplotlib.pyplot as plt
import csv


def readData(fn):
    X = []
    Y = []

    with open(fn, 'r') as file:
        reader = csv.reader(file)
        for line in reader:
            X.append(float(line[0]))
            Y.append(float(line[1]))
    return X, Y


fn1 = './output_part1.csv'
fn2 = './output_part2.csv'
fn3 = './output_part3.csv'
fn4 = './output_part4.csv'
fn5 = './output_part5.csv'

X1, Y1 = readData(fn1)
X2, Y2 = readData(fn2)
X3, Y3 = readData(fn3)
X4, Y4 = readData(fn4)
X5, Y5 = readData(fn5)

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams['font.size'] = 13

plt.plot(Y1, X1, label='[**]->[ZH]')
plt.plot(Y2, X2, label='[ZH]->[HY]')
plt.plot(Y3, X3, label='[HY]->[YH]')
plt.plot(Y4, X4, label='[YH]->[HZ]')
plt.plot(Y5, X5, label='[HZ]->[**]')

plt.annotate("",
             xy=(99500, 145000), xycoords='data',
             xytext=(100000, 144700), textcoords='data',
             size=15, va="center", ha="center",
             bbox=dict(boxstyle="round4", fc="w"),
             arrowprops=dict(arrowstyle="-|>",
                             connectionstyle="arc3,rad=-0.3",
                             fc="w"),)

plt.text(99650, 144910, 'Curve Dir',
         bbox={'facecolor': 'green', 'alpha': 0.5, 'pad': 5})

plt.scatter(99482.203, 144534.844,
            label="P_JD [144534.844, 99482.203]", marker='X')
plt.text(99532.203, 144584.844, 'P_JD',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
plt.scatter(99986.531, 144506.156,
            label="P_ZH [144506.156, 99986.531]", marker='X')
plt.text(99936.531, 144556.156, 'P_ZH',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
plt.scatter(99886.875, 144514.203,
            label="P_HY [144514.203, 99886.875]", marker='X')
plt.text(99836.875, 144464.203, 'P_HY',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
plt.scatter(99562.336, 144644.062,
            label="P_QZ [144644.062, 99562.336]", marker='X')
plt.text(99612.336, 144644.062, 'P_QZ',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
plt.scatter(99341.055, 144914.672,
            label="P_YH [144914.672, 99341.055]", marker='X')
plt.text(99391.055, 144914.672, 'P_YH',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
plt.scatter(99303.461, 145007.312,
            label="P_HZ [145007.312, 99303.461]", marker='X')
plt.text(99353.461, 144997.312, 'P_HZ',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})

plt.legend()
plt.xlabel('Y(m)')
plt.ylabel('X(m)')
plt.grid(ls='--', alpha=0.5)
plt.title("Schematic Diagram of Setting Out Curve")

plt.show()

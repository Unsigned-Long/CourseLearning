# coding:utf-8

import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams['font.size'] = 13

plt.plot([99986.531, 99482.203, 99303.461], [
         144506.156, 144534.844, 145007.312], linestyle='--', c='r')

plt.scatter(99986.531, 144506.156, c='r', label="P_ZH [144506.156, 99986.531]")
plt.text(99936.531, 144556.156, 'P_ZH',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
plt.scatter(99482.203, 144534.844, c='g', label="P_JD [144534.844, 99482.203]")
plt.text(99532.203, 144584.844, 'P_JD',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})
plt.scatter(99303.461, 145007.312, c='b', label="P_HZ [145007.312, 99303.461]")
plt.text(99353.461, 144997.312, 'P_HZ',
         bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 5})

plt.annotate("",
             xy=(99500, 144800), xycoords='data',
             xytext=(99800, 144600), textcoords='data',
             size=15, va="center", ha="center",
             bbox=dict(boxstyle="round4", fc="w"),
             arrowprops=dict(arrowstyle="-|>",
                             connectionstyle="arc3,rad=-0.3",
                             fc="w"),)

plt.text(99650, 144710, 'Curve Dir',
         bbox={'facecolor': 'green', 'alpha': 0.5, 'pad': 5})

plt.legend()
plt.xlabel('Y(m)')
plt.ylabel('X(m)')
plt.grid(ls='--', alpha=0.5)
plt.title("Schematic Diagram of Setting Out Curve")

plt.show()

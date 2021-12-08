import matplotlib.pyplot as plt
plt.plot([0, 4, 4, 0, 0], [0, 0, 4, 4, 0], c='r')
plt.plot([5, 7, 7, 5, 5], [0, 0, 7, 7, 0], c='r')
plt.plot([2, 6, 6, 2, 2], [0, 0, 6, 6, 0], c='g')
plt.Rectangle(xy=(0, 0), width=10, height=10)
# 2 0, 6 0, 6 6, 2 6, 2 0
plt.show()

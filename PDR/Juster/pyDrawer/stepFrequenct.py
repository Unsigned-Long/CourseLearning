import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import numpy as np

plt.rcParams["font.family"] = "Ubuntu Mono"
plt.rcParams["font.size"] = 13

fig = plt.figure(figsize=plt.figaspect(0.5))
ax = fig.add_subplot(1, 2, 1, projection='3d')

# 0.371, 0.227 and 1
a = 0.371
b = 0.227
c = 1.0
H = np.arange(1.4, 2.0, 0.01)
SF = np.arange(0.0, 4.0, 0.01)
H, SF = np.meshgrid(H, SF)
SL = (0.7+a*(H-1.75)+b*(SF-1.79)*H/1.75)*c

# Plot the surface.
surf = ax.plot_surface(H, SF, SL, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
ax.plot_wireframe(H, SF, SL, rstride=10, cstride=5, linewidth=1.0)
# Customize the z axis.
# ax.set_zlim(-1.01, 1.01)
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
ax.zaxis.set_major_formatter('{x:.02f}')

# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)
ax.set_xlabel('Height')
ax.set_ylabel('Step Frequency')
ax.set_zlabel('Step Length')
ax.set_title('SL with H and SF')

ax = fig.add_subplot(1, 2, 2, projection='3d')
surf = ax.plot_surface(H, SF, SL*SF, cmap=plt.get_cmap('rainbow'),
                       linewidth=0, antialiased=False)
ax.plot_wireframe(H, SF, SL*SF, rstride=10, cstride=5, linewidth=1.0)
# Customize the z axis.
# ax.set_zlim(-1.01, 1.01)
ax.zaxis.set_major_locator(LinearLocator(10))
# A StrMethodFormatter is used automatically
ax.zaxis.set_major_formatter('{x:.02f}')
fig.colorbar(surf, shrink=0.5, aspect=5)

ax.set_xlabel('Height')
ax.set_ylabel('Step Frequency')
ax.set_zlabel('Speed')
ax.set_title('Speed with H and SF')

plt.show()

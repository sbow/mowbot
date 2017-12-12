# mainLidar.py
import parseLidarFunc
import matplotlib.pyplot as plt
import numpy as np
import os

os.system('rostopic echo -n 1 /scan/ranges > scanOut.csv')
angles, rpData, rpValidData = parseLidarFunc.getRpData()

fig = plt.figure()
ax2 = fig.add_subplot(111, projection='polar')
line1, = ax2.plot(angles, rpValidData, 'r')
ax2.set_rmax(4)
plt.show()


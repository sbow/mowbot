# mainLidar.py
import parseLidarFunc
import matplotlib.pyplot as plt
import numpy as np
import os

os.system('rostopic echo -n 1 /scan/ranges > scanOut.csv')
angles, rpData, rpValidData = parseLidarFunc.getRpData()

fig = plt.figure()
ax2 = fig.add_subplot(111, projection='polar')
#line1, = ax2.plot(angles, rpData, 'r', angles, rpValidData, 'g')
line1, = ax2.plot(angles, rpValidData, 'r')
print len(rpValidData)
print len(rpData)
print type(rpValidData)
print type(rpData)
ax2.set_rmax(4)
#plt.ion()
plt.show()
#fig.canvas.draw()


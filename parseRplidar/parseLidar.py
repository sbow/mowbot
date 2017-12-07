# parseLidar.py
# Shaun Bowman
# 2017/12/06

import numpy as np
import matplotlib.pyplot as plt

FILE = 'scanOut.csv'
INF  = '999.99'  # define infinity, in meters

# constants from "rostopic echo -n 1 /scan":
angle_min       =  -3.12413907051
angle_max       =   3.14159274101
angle_increment =   0.0174532923847
time_increment  =   2.21559616875e-07
scan_time       =   7.95399027993e-05
range_min       =   0.15000000596
range_max       =   8.0

# Calculate number of datum in given scan:
n_datum =   int(round((abs(angle_min) + angle_max)/angle_increment)) + 1

# Calculate angles
angles = np.arange(angle_min, angle_max+angle_increment, angle_increment)

# Parse ROS output file
with open(FILE) as f:
    content = f.readlines()
content = [x.strip() for x in content]  # remove new-line characters
content = [x[1:-1] for x in content]    # pop first and last characters '[' & ']'
content = [x.replace('inf', INF) for x in content]  # replace 'inf' string wtih approximation
content = [x for x in content if x != '-'] # remove rows containing only '-'; a delimineter from ROS echo

#print content[1]

# Calculate number of scans
n_scans = len(content)

# create empty array to store measurements
data = np.zeros( (n_scans, n_datum) )

# for each measurement, divide measurements into cells, removedelimineter, turn
# to float:
for x in range(n_scans):
    data[x] = [float(value) for value in content[x].split(',')] # seperate elements by ',' & turn into float

avg_data = np.zeros( n_datum )
# below does not work well because it amlifies infinite measuremnts
#avg_data = np.sum( data, axis=0 ) / n_scans
for x in range(n_datum):
    for y in range(n_scans):
        n_valid = 0
        avg_datum = 0
        datum = data[y][x]
        if datum < range_max and datum > range_min:
            n_valid = n_valid + 1
            avg_datum = avg_datum + datum
    if n_valid > 0:
        avg_datum = avg_datum / float( n_valid )
        avg_data[x] = avg_datum
    # If all measurements are invalid at given angle use last datum at given
    # angle
    else:
        avg_data[x] = datum

# Detect Edges
# edge == >EDGE with adjacent datum << than 999.99
EDGE = 30
edge_datum = np.zeros( 0 )
edge_angle = np.zeros( 0 )
for x in range(n_datum):
    datum = avg_data[x]
    if datum > EDGE:
        if x > 0:
            if avg_data[x-1] < EDGE:
                edge_datum = np.append( edge_datum, avg_data[x-1] )
                edge_angle = np.append( edge_angle, angles[x-1] )
            if x < n_datum-1:
                if avg_data[x+1] < EDGE:
                    edge_datum = np.append( edge_datum, avg_data[x+1] )
                    edge_angle = np.append( edge_angle, angles[x+1] )

# Detect Edges v2 - not done, filter out edges if radial distance to valid
# neibor is less than threshold



# output:
print len(angles)
print len(data[0])

ax = plt.subplot(211, projection='polar')
# plot average and single solution; note average is smoother
ax.plot(angles, avg_data, 'b')
#ax.set_rmax(range_max)
ax.set_rmax(4)
ax = plt.subplot(212, projection='polar')
ax.plot(angles, data[0], 'r')
#ax.set_rmax(range_max)
ax.set_rmax(4)
plt.show()

ax2 = plt.subplot(111, projection='polar')
ax2.plot(angles, avg_data, 'b', edge_angle, edge_datum, 'r*')
ax2.set_rmax(4)
plt.show()

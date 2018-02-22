# parseLidar.py
# Shaun Bowman
# 2017/12/06

import numpy as np
import matplotlib.pyplot as plt
import math

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
# neibor is less than threshold.
# This filters noise in reading where some reflections read inf when they are
# should not, but trys to retain edges of real objects in FOV
#EDGE = 30   # m, if neighbor more than 30 m away, flag as an edge
#FILTER_NEAREDGE_D = 0.5  #m, do not call edge if another edge is detected within a certain angle
#FILTER_NEAREDGE_A = 2.0*3.14/180.0 #rad, d 
#n_filter_edge_a = int(round(FILTER_NEAREDGE_A / angle_increment))
#edge_datum = np.zeros( 0 )
#edge_angle = np.zeros( 0 )
#for x in range(n_datum):
#    datum = avg_data[x]
#    if datum > EDGE:
#
#        data_prev  = avg_data[(x-1)%n_datum]    # note: unusual index lookup to allow wrap-around of array
#        data_next  = avg_data[(x+1)%n_datum]
#        angle_prev = angles[(x-1)%n_datum]
#        angle_next = angles[(x+1)%n_datum]
#        
#        b_nearPrevEdge = False
#        b_nearNextEdge = False
#        b_nextObj  = False
#        b_prevObj  = False
#
#        if data_prev < EDGE:
#            b_prevObj = True
#
#        if data_next < EDGE:
#            b_nextObj = True
# 
#        for i in range(n_filter_edge_a): 
#            if abs(avg_data[(x+1+i)%n_datum]) < EDGE:
#                b_nearNextEdge = True
#        for i in range(n_filter_edge_a): 
#            if abs(avg_data[(x-1-i)%n_datum]) < EDGE:
#                b_nearPrevEdge = True
#
#        if b_prevObj:
#            if not (b_nearPrevEdge and b_nearNextEdge):
#                edge_datum = np.append( edge_datum, data_prev )
#                edge_angle = np.append( edge_angle, angle_prev )
#        if b_nextObj:
#            if not (b_nearPrevEdge and b_nearNextEdge):
#                edge_datum = np.append( edge_datum, data_next )
#                edge_angle = np.append( edge_angle, angle_next )
#
# polar coordinantes distance calculation between two points
# note: cos(theta1 - theta2) ~ 1 for lidar
# algo: if d(r,theta1 - r,theta3) < thresh; ignore r,theta2
#       else: r,theta1 and r,theta3 are edges of surfaces/objects
# note: this algo likely becomes less accurate as r,theta(1,2,3) become  co-linear
# with sensor location; but this is also a fundamental limitation of LIDAR scan
# d = sqrt( r1^2 + r2^2 - 2*r1*r2*cos(theta1 - theta2) )
EDGE        = 30
d_thresh    = 1.0
edge_datum  = np.zeros( 0 )
edge_angle  = np.zeros( 0 )
d_datums    = np.zeros( n_datum )
delta_r     = np.zeros( n_datum )   # vector that when added to prior datum equals
delta_a     = np.zeros( n_datum )   # current datum
filt_data   = np.zeros( n_datum )
n_width     = 6    # window width

for i in range(n_datum):

    prev_r = avg_data[(i-1)%n_datum]
    curr_r = avg_data[(i)%n_datum]
    next_r = avg_data[(i+1)%n_datum]
    prev_a = angles[(i-1)%n_datum]
    curr_a = angles[(i)%n_datum]
    next_a = angles[(i+1)%n_datum]

    d_datums[i]  = math.pow( math.pow( prev_r, 2.0) + math.pow( next_r, 2.0) - 2.0 * prev_r * next_r * math.cos( prev_a - next_a ), 0.5)
    
    delta_r[i]  = math.pow( math.pow( prev_r, 2.0) + math.pow( curr_r, 2.0) - 2.0 * prev_r * curr_r * math.cos( prev_a - curr_a ), 0.5)
    delta_a[i]  = math.atan2( prev_r * math.sin( prev_a ) - curr_r * math.sin(curr_a), prev_r * math.cos( prev_a ) - curr_r * math.cos( curr_a ))

    filt_data[i] = avg_data[i]

for i in range(n_datum):

    prev_r = filt_data[(i-1)%n_datum]
    curr_r = filt_data[(i)%n_datum]
    next_r = filt_data[(i+1)%n_datum]
    prev_a = angles[(i-1)%n_datum]
    curr_a = angles[(i)%n_datum]
    next_a = angles[(i+1)%n_datum]

    curr_del_r  =   delta_r[(i)%n_datum]
    next_del_r  =   delta_a[(i+1)%n_datum]

    if curr_r > EDGE:
        i_remove = [  ]

        for x in range(n_width):
            if filt_data[(i+x)%n_datum] > EDGE:
                i_remove.append( (i+x)%n_datum )
                if filt_data[(i+x+1)%n_datum] < EDGE:
                    #i_remove.append( (i+x)%n_datum )
                    x = n_width-1 # end list (exit for statement)
                    #print angles[ (i+x)%n_datum ]
                    #print (i+x)%n_datum
                    #print avg_data[(i+x)%n_datum]
                    #print 'END'
        #print 'FIN'
        if len(i_remove) > 0 and len(i_remove) < n_width:
            filt_edgestart_i = (i_remove[0]-1)%n_datum
            filt_edgeend_i = (i_remove[-1]+1)%n_datum
            filt_edgestart_r = filt_data[ filt_edgestart_i ]
            filt_edgeend_r = filt_data[ filt_edgeend_i ]
            filt_edgestart_x = filt_edgestart_r * math.cos( angles[filt_edgestart_i] )
            filt_edgestart_y = filt_edgestart_r * math.sin( angles[filt_edgestart_i] )
            filt_edgeend_x = filt_edgeend_r * math.cos( angles[filt_edgeend_i] )
            filt_edgeend_y = filt_edgeend_r * math.sin( angles[filt_edgeend_i] )
            
            filt_fill_del_x = (filt_edgeend_x - filt_edgestart_x) / (1+len(i_remove))
            filt_fill_del_y = (filt_edgeend_y - filt_edgestart_y) / (1+len(i_remove))

            for x in range(len(i_remove)):
                filt_x = (x+1)*filt_fill_del_x + filt_edgestart_x
                filt_y = (x+1)*filt_fill_del_y + filt_edgestart_y
                filt_r = math.pow( filt_x*filt_x + filt_y*filt_y, 0.5)
                filt_data[i_remove[x]] = filt_r
                


print len(data[0])

#ax = plt.subplot(211, projection='polar')
# plot average and single solution; note average is smoother
#ax.plot(angles, avg_data, 'b*')
#ax.set_rmax(range_max)
#ax.set_rmax(4)
#ax = plt.subplot(212, projection='polar')
#ax.plot(angles, data[0], 'r')
#ax.set_rmax(range_max)
#ax.set_rmax(4)
#plt.show()

ax2 = plt.subplot(111, projection='polar')
ax2.plot(angles, avg_data, 'b', angles, filt_data, 'r', angles, d_datums, 'g*')
ax2.set_rmax(4)
plt.show()

# parseLidarFunc.py
# Shaun Bowman
# 2017/12/09

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


def getRpData():
    # Parse ROS output file
    with open(FILE) as f:
        content = f.readlines()
    content = [x.strip() for x in content]  # remove new-line characters
    content = [x[1:-1] for x in content]    # pop first and last characters '[' & ']'
    content = [x.replace('inf', INF) for x in content]  # replace 'inf' string wtih approximation
    content = [x for x in content if x != '-'] # remove rows containing only '-'; a delimineter from ROS echo
    
    # Calculate number of scans
    n_scans = len(content)
    
    # create empty array to store measurements
    data = np.zeros( (n_scans, n_datum) )
    
    # for each measurement, divide measurements into cells, removedelimineter, turn
    # to float:
    for x in range(n_scans):
        data[x] = [float(value) for value in content[x].split(',')] # seperate elements by ',' & turn into float
    
    # below does not work well because it amlifies infinite measuremnts
    #avg_data = np.sum( data, axis=0 ) / n_scans
    avg_data = np.zeros( n_datum )
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
    
    # Begin Filter - Eliminate false inf & create filtered dataset
    EDGE        = 30
    d_thresh    = 1.0
    edge_datum  = np.zeros( 0 )
    edge_angle  = np.zeros( 0 )
    delta_r     = np.zeros( n_datum )   # vector that when added to prior datum equals
    delta_a     = np.zeros( n_datum )   # current datum
    filt_data   = np.zeros( n_datum )
    filt_valid_data = np.zeros( n_datum )
    n_width     = 4    # window width
    
    for i in range(n_datum):
    
        prev_r = avg_data[(i-1)%n_datum]
        curr_r = avg_data[(i)%n_datum]
        next_r = avg_data[(i+1)%n_datum]
        prev_a = angles[(i-1)%n_datum]
        curr_a = angles[(i)%n_datum]
        next_a = angles[(i+1)%n_datum]
        
        delta_r[i]  = math.pow( math.pow( prev_r, 2.0) + math.pow( curr_r, 2.0) - 2.0 * prev_r * curr_r * math.cos( prev_a - curr_a ), 0.5)
        delta_a[i]  = math.atan2( prev_r * math.sin( prev_a ) - curr_r * math.sin(curr_a), prev_r * math.cos( prev_a ) - curr_r * math.cos( curr_a ))
    
        filt_data[i] = avg_data[i]
        filt_valid_data[i] = avg_data[i]

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
                        x = n_width-1 # end list (exit for statement)
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
                    filt_valid_data[i_remove[x]] = filt_r

    # Filter out remaining valid signals using a "use last good" strategy >
    # hack.def
    temp_datum = 0
    for x in range(len(filt_data)):
        curr_datum = filt_data[x]
        if curr_datum > range_min:
            if curr_datum < range_max:
                temp_datum = curr_datum
        if curr_datum < range_min:
            filt_valid_data[x] = temp_datum
        elif curr_datum > range_max:
            filt_valid_data[x] = temp_datum

        #filt_valid_data[x] = max( [0.15, min( [8.0, filt_data[x] ] ) ] )

    return(angles, filt_data, filt_valid_data, avg_data)

#   PolySolve.py
#   Shaun Bowman
#   February 24 2018
#   
#   Solves for coefficients best fitting x,y measurements
#   
import numpy as np

# pretend measurements:
# want to find path given measurements (xi,yi)

# y1 = b0 + b1x1 + b2x1^2 + b3x1^3... ect
n_meas = 5
b0_meas = 4.1
b1_meas = 2.1

e_noise = [np.random.rand(1,1) for i in range(n_meas)]
x_meas = np.array([4.5, 5.5, 5.0, 5.5, 6.0])
y_meas = []
for i in range(n_meas):
    y_meas.append( b0_meas + b1_meas*x_meas[i] )

print('{} {} {} {} {}'.format(y_meas[0],y_meas[1],y_meas[2],y_meas[3],y_meas[4]))


# solving 2nd degree polynomial case:
#    moore penrose pseudo inverse method
#    vector B hat = inv( X_traspose * X)*X_transpose*y_meas
#    where vector B hat contains the coefficients of the
#    polynomial according to the least squares method
#    note: for time series, can re-use many peices of data in
#    the matrix X & XtransX as only one element is new and
#    another is removed.
X = np.array( np.transpose( np.array([ np.ones(n_meas), x_meas]) ))
XtransX= np.array([(n_meas, np.sum(x_meas)),(np.sum(x_meas), np.sum( \
    np.power(x_meas,2)))])

inv_XtransX = np.linalg.inv( XtransX )
det_XtransX = np.linalg.det( XtransX ) #not used

bi_leastsqfit = np.matmul(np.matmul(inv_XtransX, np.transpose(\
    X )), y_meas)

print('b0: {} b1: {}'.format(bi_leastsqfit[0], bi_leastsqfit[1]))

import numpy as np
from scipy import misc
import scipy
import matplotlib.pyplot as plt

import scipy.optimize as optimization

plt.ion()
plt.close('all')

f = misc.imread("../Images/download.jpg", flatten=True)
f = misc.imresize(f, (300, 400))

def eval_poly(row_num):
    row_num /= 1
    theta = np.array([
        row_num ** 0.0 ,
        row_num ** 1.0 ,
        row_num ** 2.0 ,
        row_num ** 3.0 ,
    ], dtype=np.float)
    PA = P.dot(A)
    column = PA.dot(theta)
    # print(column)
    column /= column[2]
    x, y, _ = column
    return x, y


def plot_poly(As, P):
    g = f.copy()
    rows, columns = f.shape

    row_num = 0
    while(True):
        try:
            x, y = eval_poly(row_num)
            x_ = int(x)
            y_ = int(y)
            # print("time", row_num, "x", x, "y", y);
            g[x_][y_] = 255
            row_num += 1
        except Exception as e:
            print(e)
            break

    plt.imshow(g)
    ax.imshow(g)

# assumption nr 1, the cubic is on a plane
a0z = a1z = a2z = a3z = 0

# start from middle of img
a0x, a0y = 0, 200

a1x, a1y = 1, 0
a2x, a2y = 0, 0.002
a3x, a3y = 0, -0.00001

period = 100

A1 = np.array([
    0,   1,   0,   0,
    a0y, a1y, a2y, a3y,
    0, 0, 0, 0,
    1, 0, 0, 0,
], dtype=np.float).reshape(4, 4)

a0y += period

A2 = np.array([
    0,   1,   0,   0,
    a0y, a1y, a2y, a3y,
    0, 0, 0, 0,
    1, 0, 0, 0,
], dtype=np.float).reshape(4, 4)

# represent the focal length of the camera in terms
# of pixel dimensions in the x and y direction respectively.
ax = f * mx
ay = f * my

# Similarly, x is the principal point in terms of pixel dimensions,
# with coordinates x0 = mx*px and y0 = my*py

# skew
s = 0

"""
P = KR[I | − C ], has 10 degrees of freedom:
4 for K (the elements s, f, px, py),
3 for R,
and 3 for C.

K are called the internal camera parameters.
R, C external parameters.
"""

K = np.array(
    [ax, s,  x0,]
    [0,  ay, y0,]
    [0,  0,  1, ]
)

# 3 dof rotate(rx, ry, rz)
R = np.array(
    [1, 0, 0,]
    [0, 1, 0,]
    [0, 0, 1,]
)

IC = np.array(
    [1, 0, 0, x0]
    [0, 1, 0, y0]
    [0, 0, 1, z0]
)
P = K*R*IC

# A1 has
A1 = np.array([
    0,   1,   0,   0,
    a0y, a1y, a2y, a3y,
    0, 0, 0, 0,
    1, 0, 0, 0,
], dtype=np.float).reshape(4, 4)

fig = plt.figure()
ax = fig.add_subplot(111)
# plot_poly(A, P)

plot_poly(A1, P)
plot_poly(A2, P)

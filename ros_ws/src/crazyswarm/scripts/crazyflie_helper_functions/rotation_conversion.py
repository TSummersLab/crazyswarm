#!/usr/bin/env python
'''
Functions to convert from one rotation representation to another.

Author:
Sleiman Safaoui
Github:
The-SS
Email:
snsafaoui@gmail.com
sleiman.safaoui@utdallas.edu

Date:
Oct 20, 2018

Supported conversions:
- Quaternion to Rotation Matrix (Q2R)
- Quaternion to Euler Angles (Q2EA)
'''

import numpy as np
import math

def Q2R (Q):
    '''
    transforms the quaternion Q to a rotation matrix (3x3)
    input:
        - Q: quaternion 3x1 list/array represented as (x,y,z,w). (!) w is the last variable, not the first (!)
    output:
        - RM: rotation matrix. 3x3 np.array
    '''
    x = Q[0]
    y = Q[1]
    z = Q[2]
    w = Q[3]

    RM = np.zeros([3,3])
    RM[0][0] = 1-2*y**2-2*z**2
    RM[0][1] = 2*x*y-2*z*w
    RM[0][2] = 2*x*z+2*y*w
    RM[1][0] = 2*x*y+2*z*w
    RM[1][1] = 1-2*x**2-2*z**2
    RM[1][2] = 2*y*z-2*x*w
    RM[2][0] = 2*x*z-2*y*w
    RM[2][1] = 2*y*z+2*x*w
    RM[2][2] = 1-2*x**2-2*y**2

    return RM

def Q2EA(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return np.array([X, Y, Z])

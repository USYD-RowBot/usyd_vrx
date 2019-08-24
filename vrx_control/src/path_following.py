#!/usr/bin/env python
from math import sqrt, atan2, cos, sin
import math
import shapely
import shapely.geometry
import numpy as np

def Carrotvtp(wx,wy,tx,ty,px,py,delta):
    """This algorithm is an implementation of the Carrot Chasing Algorithm
    as outlined in Niu et al. 2016 in 'Efficient Path Following Algorithm for Unmanned Surface Vehicle'.
    This algorithm places a Virtual Target Point (VTP) on the line connecting the current two waypoints.
    This VTP is always a set distance 'delta' closer to the target waypoint than the Robot is.

    Args:
        wx (float): X coordinate of the origin
        wy (float): Y coordinate of the origin
        tx (float): X coordinate of the target
        ty (float): Y coordinate of the target
        px (float): X coordinate of the robot
        py (float): Y coordinate of the robot
        delta (float): Distance ahead of the closest point on the line to target

    Returns:
        [xt, yt]: (float, float) The virtual target point

    """
    Ru = sqrt((px-wx)**2+(py-wy)**2)    #  Ru is the distance between the starting waypoint and the Robot
    theta = atan2((ty-wy),(tx-wx))      #  Theta is the angle of the line between the waypoints
    thetaU = atan2((py-wy),(px-wx))     #  ThetaU is the angle from the starting waypoint to the Robot
    beta = theta - thetaU               #  Beta is the angle inbetween
    R = Ru*cos(beta)                    #  R is the distance between the starting waypoint and the closest point on the line to the Robot
    xt = (R+delta)*cos(theta)           #  xt is the VTP, places delta units further along the waypoint line
    yt = (R+delta)*sin(theta)           #  yt is the VTP is placed delta units further along the waypoint line
    return [xt,yt]

def NLGLvtp(wx,wy,tx,ty,px,py,L):
    """ This algorithm is an implementation of the Nonlinear Guidance Law (NLGL)
    as outlined in  Niu et al. 2016 in 'Efficient Path Following Algorithm for Unmanned Surface Vehicle'
    This algorithm works by setting a Virtual Target Point at the intersection of a user-defined circle
    around the Robot, and the straight line connecting the two waypoints.
    When the path is out of range of the user-defined circle, the Robot is instructed to go back to the
    starting waypoint.
    This waypoints offers accurate tracking in the presence of disturbances -  perfect for mapping

    Args:
        wx (float): X coordinate of the origin
        wy (float): Y coordinate of the origin
        tx (float): X coordinate of the target
        ty (float): Y coordinate of the target
        px (float): X coordinate of the robot
        py (float): Y coordinate of the robot
        L (float): NLGL radius

    Returns:
        [xt, yt]: (float, float) The virtual target point
    """
    if(tx==wx):
        A = 1
        B = -2*py
        C = py**2 + wx**2 - 2*px*wx + px**2 - L**2
        Det = B**2 - 4 * A * C  # The determinant of the quadratic equation
        if(Det>0):
            x1 = wx
            x2 = wx
            y1 = (-B + sqrt(Det)) / (2*A)  # Quadratic Equation: The first x solution
            y2 = (-B - sqrt(Det)) / (2*A)  # Quadratic Equation: Second x solution
            D1 = (ty - y1)**2 + (tx - x1)**2
            D2 = (ty - y2)**2 + (tx - x2)**2
            if(D1<D2):
                xt = x1
                yt = y1
            else:
                xt = x2
                yt = y2
        elif(Det==0):
            xt = wx
            yt = (-B)/(2*A)
        else:                           # CHANGE THIS TO CLOSEST POINT ON LINE
            xt = wx
            yt = wy

    else:
        m = (ty - wy)/(tx - wx)         # This is the gradient of the line connecting the two waypoints.
        cf = -m*wx + wy - py            # This is a value used multiple times.
        A = 1 + m**2                    # The coefficient of x^2 in the quadratic equation
        B = -2*px+2*m*cf                # The coefficient of x^1 in the quadratic equation
        C = px**2 - L**2 + cf**2        # The coefficient of x^0 in the quadratic equation
        Det = B**2 - 4*A*C              # The determinant of the quadratic equation
        if (Det > 0):                   # If there are two real solutions
            x1 =(-B+sqrt(Det))/(2*A)    # Quadratic Equation: The first x solution
            x2 = (-B-sqrt(Det))/(2*A)   # Quadratic Equation: Second x solution
            y1 = m*x1-m*wx+wy           # The 1st and 2nd y solutions
            y2 = m*x2-m*wx+wy

            D1 = (ty-y1)**2+(tx-x1)**2  # Distance from the VTP and the target waypoint
            D2 = (ty-y2)**2+(tx-x2)**2
            # Pick the VTP that is closest to the target waypoint
            if (D1<D2):
                xt = x1
                yt = y1
            else:
                xt = x2
                yt = y2
        elif( Det==0 ):                   # If there is only one solution
            xt = (-B)/(2*A)
            yt = m*xt-m*wx+wy
        else:                           # If there are no real solutions - go directly to line
            # [xt, yt] = Carrotvtp(wx,wy,tx,ty,px,py,0)   #  Carrot with 0 delta value - the closest point on the line
            line = shapely.geometry.LineString([(wx, wy), (tx, ty)])  # Create a shapely line out of the waypoint line
            vessel = shapely.geometry.Point((px, py))  # Create the vessel
            tp = np.array(line.interpolate(line.project(vessel)))  # the new target point
            xt = tp[0]
            yt = tp[1]
    return [xt, yt]                  # Return the VTP


def purepursuit(xt,yt,px,py):
    """Finds the angle to a target point from the current location

    Args:
        xt (type): X coordinate of the target
        yt (type): Y coordinate of the target
        px (type): X coordinate of the robot
        py (type): X coordinate of the robot

    Returns:
        float: the angle to aim for

    """
    # Test for vertical line
    if (xt == px):
        if (yt > py):
            theta = math.pi / 2
        else:
            theta = -math.pi / 2
    else:
        theta = math.atan2((yt - py), (xt - px))
    return theta

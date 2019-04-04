# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 20:37:14 2019

@author: Spencer
"""
import numpy as np
from matplotlib import pyplot as plt
from math import *



def findDistance(bounds,xpos,ypos,psi):
    
    m = np.zeros((bounds.shape[0],1))
    for index in range(0,bounds.shape[0]):
        m[index] = (bounds[index][1]-bounds[index-1][1])/(bounds[index][0]-bounds[index-1][0])
    
    bound_segments = np.append(bounds,m,axis=1)
    # Uses point-slope form to find intersection of ray (fish heading) and the 
    # bounds of the tank. Assumes psi is between 0 and 2pi
    
    # Create ray describing current heading
    if psi<0:
        psi += 2*pi
    m_ray = tan(psi)
    
    x_intersect_arr = np.array([])
    y_intersect_arr = np.array([])
    
    for index in range(0,bound_segments.shape[0]):
        # Loop through each boundary segment and find intersection point
#        P1x = bound_segments[index][0]
#        P1y = bound_segments[index][1]
#        m_bound = bound_segments[index][2]
#        
#        
#        x_intersect = (m_ray*P0x - m_bound*P1x + P1y - P0y)/(m_ray-m_bound)
#        y_intersect = m_ray*(x_intersect-P0x) + P0y
        
        # Shortcut because we know the tank boundaries will always be either 
        # vertical or horizontal. 
        if abs(bound_segments[index][2]) == 0:
            # Horizontal line described by y = num
            y_intersect = bound_segments[index][1] # will intersect at this y value
            x_intersect = (y_intersect + m_ray*xpos - ypos)/m_ray # point-slope solved for x
        elif isinf(bound_segments[index][2]):
            # Vertical line described by x = num
            x_intersect = bound_segments[index][0] # will intersect this x value
            y_intersect = m_ray*(x_intersect - xpos) + ypos # point-slope solved for y
        
        # Save values in array
        x_intersect_arr = np.append(x_intersect_arr,x_intersect)
        y_intersect_arr = np.append(y_intersect_arr,y_intersect)
    
    # Use intersection points to find distance
    distances = np.sqrt(np.square(x_intersect_arr-xpos)+np.square(y_intersect_arr-ypos))
    
    print(str(distances)+"\n")
    print(str(x_intersect_arr))
    print(str(y_intersect_arr))
    
    # Now we need to find the right quadrant of intersection point.
    quad_ray = floor(psi/(pi/2))+1
    if (psi == pi/2 and xpos > 0) or (psi == pi and ypos > 0) or (psi == 3*pi/2 and xpos < 0):
        # correct for weird instances where quadrant is iffy. Shouldn't ever occur
        # because it's statistically unlikely yaw will be exactly pi or something like that
        quad_ray -= 1
#    
#    if (psi == 0 and ypos < 0):
#        # see above, yes these are one-off solutions but again this shouldn't really occur
#        # in practice.
#        quad_ray = 4
        
    intersect_angles = np.arctan2(y_intersect_arr-ypos,x_intersect_arr-xpos)
    for i in range(len(intersect_angles)):
        if intersect_angles[i] < 0:
            intersect_angles[i] += 2*pi
            
    quad_intersections = np.floor(intersect_angles/(pi/2))+1
    
    for i in range(len(quad_intersections)):                        # not out of the fish bounds
        if ((quad_intersections[i] == quad_ray) and (abs(x_intersect_arr[i]) <= (bound_Y)) and (abs(y_intersect_arr[i]) <= (bound_X))):
            dw = distances[i]
            x_intersect_actual = x_intersect_arr[i]
            y_intersect_actual = y_intersect_arr[i]
            break
        else:
            dw = 0.1
     
    print(str(quad_ray)+"\n")
    print(str(quad_intersections)+"\n")
    print("distance = "+str(dw)+"\n")
    return x_intersect_actual, y_intersect_actual, dw

bound_X = 1.2 #x
bound_Y = 1.2 #y

# Make sure bounds are consecutive, meaning you're tracing the tank walls
# BOUNDS    =                 X                     Y
#bounds = np.array([[    -bound_length/2,    -bound_width/2],
#                   [    -bound_length/2,     bound_width/2],
#                   [     bound_length/2,     bound_width/2],
#                   [     bound_length/2,    -bound_width/2]])

bounds = np.array([[    -0.1           ,       -0.1      ],
                   [    -0.1           ,       bound_Y ],
                   [    bound_X     ,       bound_Y],
                   [    bound_X     ,       -0.1       ]])

xpos = 0.9322678419437471      
ypos = 0.01744464950181149     
psi = 2.88156941290165

x_intersect, y_intersect, dw = findDistance(bounds,xpos,ypos,psi)

plt.figure()
plt.plot(np.append(bounds[:,0],bounds[0,0]),np.append(bounds[:,1],[bounds[0,1]]))
plt.plot(x_intersect,y_intersect,'ro')
plt.plot(xpos,ypos,'bo')
plt.plot([xpos,x_intersect],[ypos,y_intersect],'b-')
plt.xlim((bounds[0,0]-0.1,bound_X+0.1))
plt.ylim((bounds[0,1]-0.1,bound_Y+0.1))

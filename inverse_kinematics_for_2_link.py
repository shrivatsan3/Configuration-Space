# -*- coding: utf-8 -*-
"""
Created on Mon Oct 21 23:26:38 2019

@author: shriv
"""
import numpy as np
def distance(p1,p2):
   """finds distance between p1 and p2""" 
   return np.sqrt(np.sum(np.power(p2-p1,2)))

def slope_line(p1,p2): #takes in two points and finds the slope of line joining them
     
    m = p2-p1
    
    if((m[0]==0) & (p2[1]>p1[1])):
        return np.pi/2
    
    elif((m[0]==0) & (p2[1]<p1[1]) ):
        return -np.pi/2    
    else:
        return np.arctan2(m[1],m[0])

orig = np.array([0,0])
end_pt = np.array([ -2, 0])

phi_1 = slope_line(orig, end_pt)

l1 = 1
l2 = 1
l3 = distance(orig, end_pt)

phi_2 = np.arccos((l1**2 + l3**2 - l2**2)/(2*l1*l3))

thetha_2 =  [-(np.pi-np.arccos((l1**2 + l2**2 - l3**2)/(2*l1*l2))), np.pi-np.arccos((l1**2 + l2**2 - l3**2)/(2*l1*l2))]

joint_coord = [np.array([l1*np.cos(phi_1 + phi_2), l1*np.sin(phi_1 + phi_2)]), np.array([l1*np.cos(phi_1 - phi_2), l1*np.sin(phi_1 - phi_2)])]

thetha_1 = [slope_line(orig, joint_coord[0]), slope_line(orig,  joint_coord[1])]

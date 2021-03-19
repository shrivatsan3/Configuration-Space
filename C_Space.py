# -*- coding: utf-8 -*-
"""
@author: Shrivatsan K Chari

Course: Algorithmic motion planning (ASEN 5519)

Program to demonstrate the construction of the Configuration Space from Workspace 
for a 2-link manipulator

input : Number of obstacles, Number of vertices each obstacles has, and coordinates of the 
vertices of each obstacle

output : Displays C-Space with its obstacles
"""
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
from matplotlib.colors import ListedColormap

class Obstacle: #defining all obstacles as objects of this class
        
    def __init__(self, n ):
       
        self.no_of_vertices = n   # user specified number of vertices for an obstacle
        self.vertices = []     # Create a list to store all vertices
       
    
    def create_obstacle(self):   # method to create the obstacle from its vertices
        
        print("\n\nenter coordinates of vertices in the following format : if vertex is (2,3)")
        print("then enter 2,3 i.e type 2 followed by a comma followed 3")
        for i in range(self.no_of_vertices):
            self.vertices.append(tuple(float(x) for x in input('enter {} vertex: '.format(i+1)).split(','))) #create a list which stores all vertices as tuples
        self.obstacle = Polygon(self.vertices) #create a Polygon object using shapely module
        
    def display_obstacle(self):
        
        obstacle_x, obstacle_y = self.obstacle.exterior.xy  #this method is defined in class Polygon
        plt.fill(obstacle_x, obstacle_y)
        
    def is_in_obstacle(self, line):
        return line.intersects(self.obstacle)   # binary predicate defined in shapely module
        

def link(l1,l2,thetha1, thetha2): 
    
    first_joint_x = l1*np.cos(thetha1) #calculate x-coordinate of first joint
    
    end_effector_x = first_joint_x + l2*np.cos(thetha1+thetha2) #calculate x-coordinate of end effector
    
    first_joint_y = l1*np.sin(thetha1) #calculate y-coordinate of first joint
    
    end_effector_y = first_joint_y + l2*np.sin(thetha1+thetha2)  #calculate y-coordinate ofend effector
    
    return  (first_joint_x, end_effector_x, first_joint_y, end_effector_y)
    

def check_collision(thetha1, thetha2):
    
    collision_flag = 0  # using a variable to keep track of collision
    
    first_joint_x, end_effector_x, first_joint_y, end_effector_y = link(l1,l2,thetha1, thetha2)
    
    first_link = LineString([(0,0),(first_joint_x,first_joint_y)]) #creating a line object from shapely module
    second_link = LineString([(first_joint_x,first_joint_y), (end_effector_x,end_effector_y)])
   
    for o in obstacles:
            
        if (o.is_in_obstacle(first_link))|(o.is_in_obstacle(second_link)):
            collision_flag = 1  #set flag if collision occurs
            
        
    return collision_flag
   
    

def make_c_space():
    """constructs the c space by defining a grid"""
    
    thetha_1 = np.arange(0,2*np.pi,0.01)  #using arange rather than linspace since arange will give an open bracket at 2*pi
    thetha_2 = np.arange(0,2*np.pi,0.01)    
    thetha1,thetha2 = np.meshgrid(thetha_1,thetha_2) #create a grid from thetha values
    
    c_space_grid = np.zeros(thetha1.shape, dtype = float)
    
    for i,x in enumerate(thetha_1):
        for j,y in enumerate(thetha_2):
            c_space_grid[j,i] = check_collision(x,y)  #check every C-Space grid point for collision
    
    return (thetha1, thetha2, c_space_grid)

def plot_c_space(thetha1, thetha2, c_space_grid):
    """Plot free space and osbtacles on the C-space grid."""
   
    background_colormap = ListedColormap(["white","red"]) #white for free space. red for obstacles
    
    plt.figure(figsize =(10,10))
    plt.pcolormesh(thetha1, thetha2, c_space_grid, cmap = background_colormap, alpha = 0.5)
    
    #plt.scatter(predictors[:,0], predictors [:,1], c = outcomes, cmap = observation_colormap, s = 50)
    
    for node in tree:
        for neighbors in tree[node]:
            plt.plot([node[0], neighbors[0]],[node[1], neighbors[1]], 'cyan')
    for i in range(len(path)-1):
        plt.plot([path[i][0], path[i+1][0]],[path[i][1], path[i+1][1]], 'b')
    plt.plot(start[0],start[1],color = 'red',marker = 'o')
    plt.plot(goal[0],goal[1],color = 'green',marker = 'o')    

    
    plt.xlabel('Thetha 1'); plt.ylabel('Thetha 2')
    
    plt.xticks(()); plt.yticks(())
    plt.xlim (0, 2*np.pi) #specifying limits for the axes
    plt.ylim (0, 2*np.pi)
    plt.axis([0, 2*np.pi, 0, 2*np.pi])
  
"""   
No_of_obstacles = int(input("\nenter number of obstacles: "))
obstacles = []
for i in range(No_of_obstacles):
    n = int(input('\nenter number of vertices in {} obstacle: '.format(i+1)))
    obstacle_element = Obstacle(n)
    obstacle_element.create_obstacle()
    obstacles.append(obstacle_element)

"""
#l1 = float(input("enter length of first link: "))

#l2 = float(input("enter length of second link: "))
    

thetha1, thetha2, c_space_grid = make_c_space()
plot_c_space(thetha1, thetha2, c_space_grid)

"""    
plt.figure(figsize =(10,10))
for o in obstacles:
    o.display_obstacle() 

"""

    
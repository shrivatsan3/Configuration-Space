# -*- coding: utf-8 -*-
"""
ASEN 5519
ALGORITHMIC MOTION PLANNING

Program to find  a path for end effector to reach its final goal

input: start, goal, obstacles

output: visual display of the link reaching its final position

@author: shrivatsan
"""

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.geometry import Point
from shapely.geometry import LineString
from matplotlib.colors import ListedColormap
from shapely.geometry import box
from shapely.ops import nearest_points


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
   
l1 = 1 #float(input("enter length of first link: "))

l2 = 1 #float(input("enter length of second link: "))

 
def make_c_space():
    """constructs the c space by defining a grid"""
    
    thetha_1 = np.arange(0,2*np.pi,0.01)  #using arange rather than linspace since arange will give an open bracket at 2*pi
    thetha_2 = np.arange(0,2*np.pi,0.01)    
    thetha1,thetha2 = np.meshgrid(thetha_1,thetha_2) #create a grid from thetha values
    
    c_space_grid = np.zeros(thetha1.shape, dtype = float)
    visited_pixels = np.zeros(thetha1.shape, dtype = float)

    for i,x in enumerate(thetha_1):
        for j,y in enumerate(thetha_2):
            
            pixel = box(x - 0.005,y - 0.005,x + 0.005,y + 0.005)    #Pixel is a box centred at thetha values with sides of length 0.01
            
            if pixel.contains(goal): # define goal pixel value as 2
                c_space_grid[j,i] = 2  
                goal_pixel_x = i
                goal_pixel_y = j
                continue
                
            if pixel.contains(start): # Find the pixel indices for start point
                start_pixel_x = i
                start_pixel_y = j
                continue
            
            c_space_grid[j,i] = check_collision(x,y)  #check every C-Space grid point for collision
    
    return (thetha_1, thetha_2, c_space_grid, visited_pixels, goal_pixel_x, goal_pixel_y, start_pixel_x, start_pixel_y)


def findNeighbors(grid, point):
    (x,y) = point
    neighbors = []
    
    if 0 < x < len(grid) - 1:
        xi = (0, -1, 1)   # this isn't first or last row, so we can look above and below
    elif x > 0:
        xi = (0, -1, -len(grid[0])+1)      # this is the last row, so we can only look above and wrap around
    else:
        xi = (0, 1, len(grid[0])-1)       # this is the first row, so we can only look below and wrap around
    
    if 0 < y < len(grid[0]) - 1:
        yi = (0, -1, 1)   # this isn't first or last column, so we can look above and below
    elif y > 0:
        yi = (0, -1)      # this is the last column
    else:
        yi = (0, 1)       # this is the first column, so we can only look below
    
    for a in xi:
        for b in yi:
            if a == b == 0:  
                continue
            if (a*b) != 0:  
                continue
            neighbors.append((x + a, y + b))
    
    return neighbors        

def wavefront_plan(pixel_value, visited_pixels):
    "populate the grid according to the wavefront planner using the breadth-first search algorithm"
    visited_pixels[goal_pixel_y,goal_pixel_x] = 1   # Start at goal.
    queue = []
    queue.append((goal_pixel_x,goal_pixel_y)) # add goal pixel to the queue
    
    while queue:
        (queue_x, queue_y) = queue[0]   
        if queue_x == start_pixel_x and queue_y == start_pixel_y :  #start found, no need to populate the remaining grid
            break
        
        neighbors = findNeighbors(pixel_value, queue[0])    #Find the nieghbors for the current pixel
        
        queue.pop(0) 
        
        
        for (x_n, y_n) in neighbors:
                if pixel_value[y_n, x_n] == 1:  #if the pixel has been visited, dont add it to the queue
                    continue
                if visited_pixels[y_n, x_n] == 0:
                    
                    queue.append((x_n, y_n))        # Assign a value 1 greater than its neighbor
                    pixel_value[y_n, x_n] = pixel_value[queue_y, queue_x] + 1
                    visited_pixels[y_n,x_n] = 1
                        
    return pixel_value, visited_pixels                   


def path_finder(robot_pos_y, robot_pos_x, pixel_value, x_centre, y_centre):
        "Find the path from start to goal"
            while(not (pixel_value[robot_pos_y, robot_pos_x] == 2)): # Stop when you reach the goal pixel 
                
                neighbors = findNeighbors(pixel_value, (robot_pos_x, robot_pos_y))
                neighbor_pixel_value = pixel_value[robot_pos_y, robot_pos_x]
                
                for n in neighbors:
                    (n_x, n_y) = n
                    
                    if pixel_value[n_y, n_x] == 1:  # pixel in obstacle. Dont add it to path
                        continue
                    elif pixel_value[n_y, n_x] == 0:    #pixel in free space which hasn't been sampled. Dont add it to path
                        continue
                    elif neighbor_pixel_value > pixel_value[n_y, n_x]: #Find the shortest path by adding the neighbor with the shortest pixel value
                        neighbor_pixel_value = pixel_value[n_y, n_x]
                        robot_pos_x = n_x 
                        robot_pos_y = n_y
                        robot_pos.append(( x_centre[robot_pos_x],y_centre[robot_pos_y]))
            
            return robot_pos




No_of_obstacles = int(input("\nenter number of obstacles: "))
obstacles = []
for i in range(No_of_obstacles):
    n = int(input('\nenter number of vertices in {} obstacle: '.format(i+1)))
    obstacle_element = Obstacle(n)
    obstacle_element.create_obstacle()
    obstacles.append(obstacle_element)


goal = Point(np.pi, 0) 
start = Point(0.0, 0.0)

thetha_1, thetha_2, c_space_grid, visited_pixels,goal_pixel_x, goal_pixel_y, start_pixel_x, start_pixel_y = make_c_space()

c_space_grid, visited_pixels = wavefront_plan(c_space_grid, visited_pixels)


robot_pos_x = start_pixel_x
robot_pos_y = start_pixel_y

robot_pos = []
robot_pos.append(( start.x,start.y))
robot_pos =  path_finder(robot_pos_y, robot_pos_x, c_space_grid, thetha_1, thetha_2)


for i in np.linspace(0, len(robot_pos)-1, 10, dtype = int):
    
    for o in obstacles:
        o.display_obstacle()
    x1,x2,y1,y2 = link(l1,l2,robot_x[i], robot_y[i])
    
    plt.plot([0,x1],[0,y1],'b')
    plt.plot([x1,x2],[y1,y2],'g')
    plt.plot(x2,y2,'ro')
    plt.draw()
    plt.pause(0.0001)
   

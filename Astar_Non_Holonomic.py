import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist
import matplotlib.patches as patches

plt.ion()

class Node:

    def __init__(self, x, y, parent,current_theta, change_theta,UL,UR, c2c, c2g, total_cost ):
        self.x = x
        self.y = y
        self.parent = parent
        self.current_theta = current_theta
        self.change_theta = change_theta
        self.UL = UL
        self.UR = UR
        self.c2c = c2c
        self.c2g = c2g
        self.total_cost = total_cost 
        
    def __lt__(self,other):
        return self.total_cost < other.total_cost

def obstaclecheck_circle(x, y, r, c):
    tot = r + c

    circle1 = ((np.square(x - 2)) + (np.square(y - 2)) <= np.square(1 + tot))
    circle2 = ((np.square(x - 2)) + (np.square(y - 8)) <= np.square(1 + tot))

    if circle1 or circle2:
        return True
    else:
        return False
    
def obstaclecheck_rectangle(r,c,x,y):
    tot = r + c
    
    rect1 = (x >= 3.75 - tot) and (x <= 6.25 + tot) and (y >= 4.25 - tot) and (y <= 5.75 + tot)
    rect2 = (x >= 7.25 - tot) and (x <= 8.75 + tot) and (y >= 2 - tot) and (y <= 4 + tot)

    if rect1 or rect2:
        return True
    else:
        return False
    
def obstaclecheck_square(x, y, r, c):
    tot = r + c
    square1 = (x >= 0.25 - tot) and (x <= 1.75 + tot) and (y >= 4.25 - tot) and (y <= 5.75 + tot)
    
    if square1:
        return True
    else:
        return False

def is_valid(x,y, r,c):
    
    if ((x - r - c <= 0) or (y - r - c <= 0) or (x + r + c >= width) or ( + r + c >= height)):
        return False
    elif obstaclecheck_circle(x, y, r, c):
        return False
    elif obstaclecheck_square(x, y, r, c):
        return False
    elif obstaclecheck_rectangle(r, c, x, y):
        return False
    else:
        return True

def is_goal(current, goal):

    dt = dist((current.x, current.y), (goal.x, goal.y))

    if dt < 0.25:
        return True
    else:
        return False

def threshold(x, y, th, theta):
    x = (round(x * 10) / 10)
    y = (round(y * 10) / 10)
    th = (round(th / theta) * theta)
    return (x, y, th)

def action_model(rpm1, rpm2):
    actions = [[rpm1, 0], [0, rpm1], [rpm1, rpm1], [0, rpm2], [rpm2, 0], [rpm2, rpm2], [rpm1, rpm2], [rpm2, rpm1]]
    return actions

def plot_curve(Xi, Yi, Thetai, UL, UR,c, plot, N_list, S_list):
    t = 0
    r = 0.038
    L = 0.354
    dt = 0.1
    cost = 0
    Xn = Xi
    Yn = Yi
    Thetan = 3.14 * Thetai / 180

    # Xi, Yi,Thetai: Input point's coordinates
    # Xs, Ys: Start point coordinates for plot function
    # Xn, Yn, Thetan: End point coordintes

    while t < 1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += r*0.5 * (UL + UR) * math.cos(Thetan) * dt
        Yn += r*0.5 * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        if  is_valid(Xn, Yn, r, c):
            if plot == 0:
                # plt.plot([Xs, Xn], [Ys, Yn], color="blue")
                c2g = dist((Xs, Ys), (Xn, Yn))
                cost = cost + c2g
                N_list.append((Xn, Yn))
                S_list.append((Xs, Ys))
            if plot == 1:
                plt.plot([Xs, Xn], [Ys, Yn], color="red")
        else:
            return None
    Thetan = 180 * (Thetan) / 3.14
    return [Xn, Yn, Thetan, cost, N_list, S_list]

def Astar_algorithm(start_node, goal_node,RPM1,RPM2,r,clearance):

    if is_goal(start_node, goal_node):
        return 1,None,None

    N_list = []
    S_list = []
    open_nodes = {}
    open_nodes[(start_node.x * 2000 + start_node.y)] = start_node
    closed_nodes = {}
    theta_threshold = 15
    open_nodes_list = []
    heapq.heappush(open_nodes_list, [start_node.total_cost, start_node])

    while (len(open_nodes_list) != 0):

        current_node = (heapq.heappop(open_nodes_list))[1]
        current_id = (current_node.x * 2000 + current_node.y)

        if is_goal(current_node, goal_node):
            goal_node.parent = current_node.parent
            goal_node.total_cost = current_node.total_cost
            print("Goal Node found")
            return 1,N_list,S_list
        
        if current_id in closed_nodes:  
            continue
        else:
            closed_nodes[current_id] = current_node
        
        del open_nodes[current_id]
        
        moves = action_model(RPM1, RPM2)

        for move in moves:
            X1 = plot_curve(current_node.x, current_node.y, current_node.current_theta, move[0], move[1],
                            clearance, 0, N_list, S_list)
            if (X1 != None):
                angle = X1[2]
                th = (round(angle / theta_threshold) * theta_threshold)
                while(th>360):
                    th = th - 360

                while(th<-360):
                    th = th + 360

                if(th>360):
                    th = th - 360
                if(th<-360):
                    th = th + 360

                if(th == 360):
                    th == 0
                
                ct = current_node.change_theta - th
                x,y,t = threshold(X1[0],X1[1],th,theta_threshold)
                c2g = dist((x,y), (goal_node.x, goal_node.y))
                new_node = Node(x,y,current_node,t,ct,move[0],move[1],current_node.c2c+X1[3],c2g,current_node.c2c+X1[3]+c2g)

                new_node_id = (new_node.x * 2000+ new_node.y)

                if not is_valid(new_node.x, new_node.y,r,clearance):
                    continue
                elif new_node_id in closed_nodes:
                    continue
                if new_node_id in open_nodes:
                    if new_node.total_cost < open_nodes[new_node_id].total_cost:
                        open_nodes[new_node_id].total_cost = new_node.total_cost
                        open_nodes[new_node_id].parent = new_node
                else:
                    open_nodes[new_node_id] = new_node
                    heapq.heappush(open_nodes_list, [ open_nodes[new_node_id].total_cost, open_nodes[new_node_id]])
            
    return 0,N_list,S_list

def Generate_Path(goal_node):  

    x_path = []
    y_path = []
    theta_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)
    theta_path.append(goal_node.current_theta)
    parent_node = goal_node.parent

    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        theta_path.append(parent_node.current_theta)
        parent_node = parent_node.parent
        
    x_path.reverse()
    y_path.reverse()
    theta_path.reverse()

    x = np.asarray(x_path)
    y = np.asarray(y_path)
    theta = np.array(theta_path)
    return x,y,theta

def plot(start_node,goal_node,x_path,y_path,N_list,S_list,RPM1,RPM2,theta_path):


    figure, axes = plt.subplots()
    axes.set(xlim=(0, 10), ylim=(0,10))
    

    circle_1 = plt.Circle((2, 2), 1, fill = 'True')
    circle_2 = plt.Circle((2, 8), 1, fill = 'True')
    
    rect1 = patches.Rectangle((0.25, 4.25), 1.5, 1.5, color ='green')
    rect2 = patches.Rectangle((3.75, 4.25), 2.5, 1.5, color ='red')
    rect3 = patches.Rectangle((7.25, 2), 1, 2, color ='yellow')


    axes.set_aspect( 'equal' )
    axes.add_artist(circle_1)
    axes.add_artist(circle_2)
    axes.add_patch(rect1)
    axes.add_patch(rect2)
    axes.add_patch(rect3)

    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")

    l = 0
    while l < len(N_list):
        plt.plot([S_list[l][0], N_list[l][0]], [S_list[l][1], N_list[l][1]], color="blue")
        l = l + 1
        #plt.pause(0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)

    plt.plot(x_path,y_path, ':r')
    timer_stop = time.time()
    
    C_time = timer_stop - timer_start
    print("The Total Runtime is:  ", C_time)

    plt.show()
    plt.pause(30)
    plt.close('all')

if __name__ == '__main__':

    width = 10
    height = 10
    robot_radius  = 0.038
    clearance = input("Enter clearance of robot ")
    clearance = float(clearance)

    Rpms = input("Enter left and right RPMs")
    RPM1,RPM2 = Rpms.split()
    RPM1 = int(RPM1)
    RPM2 = int(RPM2)

    start_coordinates = input("Enter start coordinates: ")
    start_x, start_y = start_coordinates.split()
    start_x = int(start_x)
    start_y = int(start_y)

    if not is_valid(start_x, start_y, robot_radius,clearance):
        print("In valid start node or in Obstacle space")
        exit(-1)
    
    start_theta = input("Enter Orientation of the robot at start node: ")
    start_theta = int(start_theta)

    goal_coordinates = input("Enter goal coordinates: ")
    goal_x, goal_y = goal_coordinates.split()
    goal_x = int(goal_x)
    goal_y = int(goal_y)

    if not is_valid(goal_x, goal_y, robot_radius,clearance):
        print("In valid goal node or in Obstacle space")
        exit(-1)
    
    timer_start = time.time()

    c2g = dist((start_x,start_y), (goal_x, goal_y))
    total_cost =  c2g
    start_node = Node(start_x, start_y,-1,start_theta,0,0,0,0,c2g,total_cost)
    goal_node = Node(goal_x, goal_y, -1,0,0,0,0,c2g,0,total_cost)

    flag,N_list,S_list = Astar_algorithm(start_node, goal_node,RPM1,RPM2,robot_radius,clearance)
                    
    if (flag)==1:
        x_path,y_path,theta_path = Generate_Path(goal_node)
    else:
        print("not found")
        exit(-1)

    plot(start_node,goal_node,x_path,y_path,N_list,S_list,RPM1,RPM2,theta_path)
    
                


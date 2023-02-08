import numpy as np
import math
import matplotlib.pyplot as plt
import time
import heapq
from math import dist


class Node:

    def __init__(self, x, y, parent,current_theta, change_theta, velocity, c2c, c2g, total_cost ):
        self.x = x
        self.y = y
        self.parent = parent
        self.current_theta = current_theta
        self.change_theta = change_theta
        self.velocity = velocity
        self.c2c = c2c
        self.c2g = c2g
        self.total_cost = total_cost 
        
    def __lt__(self,other):
        return self.total_cost < other.total_cost

def ObstacleMap(width,height,obs_clearance,robot_radius):

    map = np.full((height,width),0)
    r = 40
    for y in range(height):
        for x in range(width):

            t1 = (y- ( obs_clearance + robot_radius)) - ((0.316) *(x + ( obs_clearance + robot_radius))) - 173.608
            t2 = (y+ ( obs_clearance + robot_radius)) + (1.23 * (x + ( obs_clearance + robot_radius))) - 229.34 
            t3 = (y- ( obs_clearance + robot_radius)) + (3.2 * (x- ( obs_clearance + robot_radius))) - 436 
            t4 = (y+ ( obs_clearance + robot_radius)) - 0.857*(x- ( obs_clearance + robot_radius)) - 111.42 
            t5 = y + (0.1136*x) - 189.09
            
            #Circle Obstacle (Clearance)
            C = ((y -185)**2) + ((x-300)**2) - (r + obs_clearance + robot_radius)**2  
            
            #Hexagon Obstacle (Clearance)
            h1 = (y- ( obs_clearance + robot_radius)) - 0.577*(x+ ( obs_clearance + robot_radius)) - 24.97
            h2 = (y- ( obs_clearance + robot_radius)) + 0.577*(x- ( obs_clearance + robot_radius)) - 255.82
            h3 = (x- ( obs_clearance + robot_radius)) - 235 
            h6 = (x+ ( obs_clearance + robot_radius)) - 165 
            h5 = (y+ ( obs_clearance + robot_radius)) + 0.577*(x+ ( obs_clearance + robot_radius)) - 175 
            h4 = (y+ ( obs_clearance + robot_radius)) - 0.577*(x- ( obs_clearance + robot_radius)) + 55.82 
            
            #Conditions defining all points bounded by these lines are in the obstacle clearance area
            if(h1<0 and h2<0 and h3<0 and h4>0 and h5>0 and h6>0) or C<=0  or (t1<0 and t5>0 and t4>0)or (t2>0 and t5<0 and t3<0):
                map[y,x] = 1
        
        #Polygon Obstacle (clearance)
            s1 = (y-5) - ((0.316) *(x+5)) - 173.608  
            s2 = (y+5) + (1.23 * (x+5)) - 229.34 
            s3 = (y-5) + (3.2 * (x-5)) - 436 
            s4 = (y+5) - 0.857*(x-5) - 111.42 
            s5 = y + (0.1136*x) - 189.09
        
        #Circle Obstacle (clearance)
            C = ((y -185)**2) + ((x-300)**2) - (r+5)**2  
            
        #Hexagon Obstacle (clearance)
            h1 = (y-5) - 0.577*(x+5) - 24.97
            h2 = (y-5) + 0.577*(x-5) - 255.82
            h3 = (x-5) - 235 
            h6 = (x+5) - 165 
            h5 = (y+5) + 0.577*(x+5) - 175 
            h4 = (y+5) - 0.577*(x-5) + 55.82 
        
            if(h1<0 and h2<0 and h3<0 and h4>0 and h5>0 and h6>0) or C<=0  or (s1<0 and s5>0 and s4>0)or (s2>0 and s5<0 and s3<0):
                map[y,x]=1
        

            a1 = y - 0.577*x - 24.97 
            a2 = y + 0.577*x - 255.82
            a3 = x - 235 
            a6 = x - 165 
            a5 = y + 0.577*x - 175 
            a4 = y - 0.577*x + 55.82 
        
            D = ((y -185)**2) + ((x-300)**2) - (r)**2 
        
            l1 = y - ((0.316) *x) - 173.608  
            l2 = y + (1.23 * x) - 229.34 
            l3 = y + (3.2 * x) - 436 
            l4 = y - 0.857*x - 111.42 
            l5 = y + (0.1136*x) - 189.09
        
            if(a1<0 and a2<0 and a3<0 and a4>0 and a5>0 and a6>0) or D<0 or (l1<0 and l5>0 and l4>0)or (l2>0 and l5<0 and l3<0): 
                map[y,x]=2

    for i in range(400):
        map[0][i] = 1
        map[1][i] = 1
        map[2][i] = 1
        map[3][i] = 1
        map[4][i] = 1

    for i in range(400):
        map[249][i] = 1
        map[248][i] = 1
        map[247][i] = 1
        map[246][i] = 1
        map[245][i] = 1

    for i in range(250):
        map[i][0] = 1
        map[i][1] = 1
        map[i][2] = 1
        map[i][3] = 1
        map[i][4] = 1

    for i in range(250):
        map[i][399] = 1
        map[i][398] = 1
        map[i][397] = 1
        map[i][396] = 1
        map[i][395] = 1

    return map


def is_valid(x, y, obstacle_map):

    s = obstacle_map.shape

    if( x >= s[1] or x < 0 or y >=s[0]  or y < 0 ):
        return False
    
    elif (obstacle_map[y][x] == 1) or (obstacle_map[y][x] == 2):
        return False

    else:
        return True

def is_goal(current, goal):

    dt = dist((current.x, current.y), (goal.x, goal.y))

    if dt < 1.5:
        return True
    else:
        return False

def Differential_Drive_Constraints(Theta,UL,UR,dt,x,y):

    Theta_rad = 3.14 * Theta / 180
    UL = UL*2*math.pi/60
    UR = UR*2*math.pi/60

    Delta_Theta_dot = (R / L) * (UR - UL)
    Delta_Theta = Delta_Theta_dot * dt
    Theta_rad = Theta_rad + Delta_Theta

    Delta_X_dot = (0.5*R * (UL + UR) * math.cos(Theta_rad))
    Delta_Y_dot = (0.5*R * (UL + UR) * math.sin(Theta_rad))
    
    Delta_X = Delta_X_dot * dt
    Delta_Y = Delta_Y_dot * dt
    
    Delta_cost = float(math.sqrt(Delta_X ** 2 + Delta_Y ** 2))
    Vel_mag = math.sqrt((Delta_X_dot) ** 2 + (Delta_Y_dot) ** 2)
    cost = 0

    if (UL!=0 and UR!=0):
        cost = np.sqrt(6)

    Delta_cost +=cost

    Theta = 180 * (Theta_rad) / 3.14
    Delta_Theta = 180 * (Delta_Theta) / 3.14

    xc = round(x + Delta_X)
    yc = round(y + Delta_Y)
    # if is_valid(round(xc), round(yc), obstacle_map):
    #         plt.plot([x, xc], [y, yc], color="blue")

    return Delta_X,Delta_Y,Delta_Theta,Theta,Delta_cost,Vel_mag,x,y,xc,yc

def Get_Motions(RPM1, RPM2, dt, Theta,x,y):

    moves = [[0, RPM1],[RPM1, 0],[RPM1, RPM1],[0, RPM2],[RPM2, 0],[RPM2, RPM2],[RPM1, RPM2],[RPM2, RPM1]]

    motions = []
    
    for move in moves:

        UR = move[1]
        UL = move[0]

        Dx, Dy,Dtheta,theta,Dcost,Vel_mag,x,y,xc,yc = Differential_Drive_Constraints(Theta,UL,UR,dt,x,y)

        motions.append((Dx, Dy, Dtheta, theta, Dcost, Vel_mag,x,y,xc,yc))

    return motions

def Astar_algorithm(start_node, goal_node,obstacle_map):

    if is_goal(start_node, goal_node):
        return 1,None

    vectors_list = []
    vectore_list = []
    open_nodes = {}
    open_nodes[(start_node.x * 500 + start_node.y)] = start_node
    closed_nodes = {}
    open_nodes_list = []  
    all_nodes_objects = []
    heapq.heappush(open_nodes_list, [start_node.total_cost, start_node])

    while (len(open_nodes_list) != 0):

        current_node = (heapq.heappop(open_nodes_list))[1]
        all_nodes_objects.append(current_node)
        current_id = (current_node.x * 500 + current_node.y) 

        if is_goal(current_node, goal_node):
            goal_node.parent = current_node.parent
            goal_node.total_cost = current_node.total_cost
            print("Goal Node found")
            return 1,all_nodes_objects,vectors_list,vectore_list

        if current_id in closed_nodes:  
            continue
        else:
            closed_nodes[current_id] = current_node
        
        del open_nodes[current_id]

        # motions = Get_Motions(RPM1, RPM2, dt, current_node.current_theta)
        motions = Get_Motions(RPM1, RPM2, dt, current_node.current_theta,current_node.x,current_node.y)

        for move in motions:

            c2g = dist((current_node.x + move[0], current_node.y + move[1]), (goal_node.x, goal_node.y))
            new_node = Node( round(current_node.x + move[0]), round(current_node.y + move[1]), 
            current_node, move[3], move[2], move[5], current_node.c2c + move[4], c2g, current_node.c2c + move[4] + c2g)  

            new_node_id = (new_node.x * 500+ new_node.y) 

            if not is_valid(new_node.x, new_node.y, obstacle_map):
                continue
            elif new_node_id in closed_nodes:
                continue
            
            vectors_list.append((current_node.x,current_node.y))
            vectore_list.append((new_node.x,new_node.y))

            if new_node_id in open_nodes:
                if new_node.total_cost < open_nodes[new_node_id].total_cost:
                    open_nodes[new_node_id].total_cost = new_node.total_cost
                    open_nodes[new_node_id].parent = new_node
            else:
                open_nodes[new_node_id] = new_node
            heapq.heappush(open_nodes_list, [ open_nodes[new_node_id].total_cost, open_nodes[new_node_id]])

    return  0,all_nodes_objects,vectors_list,vectore_list


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

def plot_curve(Xi,Yi,Thetai,UL,UR):
    t = 0
    r = 3.8
    L = 35.4
    dt = 0.1
    Xn=Xi
    Yn=Yi
    Thetan = 3.14 * Thetai / 180

    UL = UL*2*math.pi/60
    UR = UR*2*math.pi/60
# Xi, Yi,Thetai: Input point's coordinates
# Xs, Ys: Start point coordinates for plot function
# Xn, Yn, Thetan: End point coordintes
    D=0
    while t<1:
        t = t + dt
        Xs = Xn
        Ys = Yn
        Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
        Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
        Thetan += (r / L) * (UR - UL) * dt
        if is_valid(round(Xn), round(Yn), obstacle_map):
            plt.plot([Xs, Xn], [Ys, Yn], color="blue")
            plt.pause(0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)
        
    Thetan = 180 * (Thetan) / 3.14
    return Xn, Yn, Thetan, D

def plot(start_node,goal_node,x_path,y_path,obs_space,all_nodes_objects,vectors_list,vectore_list,RPM1,RPM2,theta_path):

    
    plt.figure()
    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")

    plt.imshow(obs_space, "GnBu")
    ax = plt.gca()
    ax.invert_yaxis() 

    actions = [[0, RPM1],[RPM1, 0],[RPM1, RPM1],[0, RPM2],[RPM2, 0],[RPM2, RPM2],[RPM1, RPM2],[RPM2, RPM1]]
    for i in range(0,len(x_path)):
        for action in actions:
            X1= plot_curve(x_path[i],y_path[i],theta_path[i], action[0],action[1])
            for action1 in actions:
                X2=plot_curve(X1[0],X1[1],X1[2], action1[0],action1[1])
    
    # for i in all_nodes_objects:
    #     plt.plot(i.x,i.y,"2g-")

    
    plt.plot(x_path,y_path, ':r')
    timer_stop = time.time()
    
    C_time = timer_stop - timer_start
    print("The Total Runtime is:  ", C_time)

    plt.show()
    plt.pause(3)
    plt.close('all')


if __name__ == '__main__':

    L = 35.4
    R = 3.8
    dt = 0.1

    obs_clearance = input("Assign Clearance to the Obstacles: ")
    obs_clearance = int(obs_clearance)
    
    robot_radius = input("Enter the Radius of the Robot: ") 
    robot_radius = int(robot_radius)

    width = 400
    height = 250
    obstacle_map = ObstacleMap(width, height,obs_clearance,robot_radius)

    Rpms = input("Enter left and right RPMs")
    RPM1,RPM2 = Rpms.split()
    RPM1 = int(RPM1)
    RPM2 = int(RPM2)

    start_coordinates = input("Enter start coordinates: ")
    start_x, start_y = start_coordinates.split()
    start_x = int(start_x)
    start_y = int(start_y)

    if not is_valid(start_x, start_y, obstacle_map):
        print("In valid start node or in Obstacle space")
        exit(-1)

    start_theta = input("Enter Orientation of the robot at start node: ")
    start_theta = int(start_theta)

    goal_coordinates = input("Enter goal coordinates: ")
    goal_x, goal_y = goal_coordinates.split()
    goal_x = int(goal_x)
    goal_y = int(goal_y)

    if not is_valid(goal_x, goal_y, obstacle_map):
        print("In valid goal node or in Obstacle space")
        exit(-1)

    timer_start = time.time()

    c2g = dist((start_x,start_y), (goal_x, goal_y))

    total_cost =  c2g
    start_node = Node(start_x, start_y,-1,start_theta, 0,0,0,c2g,total_cost)
    goal_node = Node(goal_x, goal_y, -1,0,0,0,c2g,0,total_cost)
    flag,all_nodes_objects,vectors_list,vectore_list = Astar_algorithm(start_node, goal_node,obstacle_map)

    if (flag)==1:
        x_path,y_path,theta_path = Generate_Path(goal_node)
    else:
        print("not found")
        exit(-1)

    plot(start_node,goal_node,x_path,y_path,obstacle_map,all_nodes_objects,vectors_list,vectore_list,RPM1,RPM2,theta_path)

     
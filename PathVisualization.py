import numpy as np 
import matplotlib.pyplot as plt
import numpy.random as rnd




def getObstaclesCoord(map,):
    obs_x, obs_y = np.where(map == 1)

    return obs_x,obs_y

def showPath(path,map,dim):
    path = np.array(path)
    x,y = path.T
    x = np.append(np.array([0]),np.append(x,dim-1))
    y = np.append(np.array([0]),np.append(y,dim-1))
    
    plt.matshow(map)
    plt.ylabel('best path')
    plt.plot(y,x)

def updatePlot(path,map,dim,plot):
    path = np.array(path)
    x,y = path.T
    x = np.append(np.array([0]),np.append(x,dim-1))
    y = np.append(np.array([0]),np.append(y,dim-1))
    plot.clear()
    plot.plot(x,y)

    obs_x, obs_y = getObstaclesCoord(map)
    plot.plot(obs_x,obs_y, 'ro')

# function that returns the sign of an integer --> porta in PATHVISUALIZATION
def sign(x):
    if x == 0:
        return 0
    elif x>0:
        return 1
    else:
        return -1

# function used to find the next point in the segment between 2 points in a path --> porta in PATHVISUALIZATION
def next_point(current_point,x_dist,y_dist,ratio):
    
    next_point = current_point
    x_inc = sign(x_dist)
    y_inc = sign(y_dist)
    x_dist = abs(x_dist)
    y_dist = abs(y_dist)

    if y_dist*ratio >= x_dist:
        next_point[1] += y_inc
    else:
        next_point[0] += x_inc

    return next_point

def drawSegment(start,end,map):
    current_point = np.copy(start)

    x_distance = end[0]-start[0]
    y_distance = end[1]-start[1]
    if y_distance == 0:
        ratio = 10000
    else:
        ratio = abs(float(x_distance)/float(y_distance))

    map[current_point[0],current_point[1]] = 1
    while not np.array_equal(current_point,end):
        x_distance = end[0]-current_point[0]
        y_distance = end[1]-current_point[1]
        current_point = next_point(current_point,x_distance,y_distance,ratio)
        map[current_point[0],current_point[1]] = 1
    
    map[current_point[0],current_point[1]] = 1

def drawPath(path,map,dim):
    drawSegment(np.array([0,0]),path[0],map)
    for i in range(len(path)-1):
        drawSegment(path[i],path[i+1],map)
    drawSegment(path[-1],np.array([dim-1,dim-1]),map)

def generate_map(num_obstacles,dim):
    map = np.zeros((dim,dim))

    Y, X = np.ogrid[:dim, :dim]
    dist_from_center = np.sqrt((X - dim/2)**2 + (Y-dim/2)**2)
    mask = (dist_from_center <= 2)
    map[mask] = 255

    for i in range(num_obstacles-1):
        center = rnd.randint(0+(dim/5),dim-(dim/5),size=2)
        radius = rnd.randint(dim/20,dim/10)
        Y, X = np.ogrid[:dim, :dim]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)
        mask = (dist_from_center <= radius)
        map[mask] = 255

    return map
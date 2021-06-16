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


def add_steepness(map):
    map[0,0]=0; map[0,1]=0;map[0,1]=10;map[0,3]=20;map[0,4]=40;map[0,5]=60;map[0,6]=100;map[0,7]=200;map[0,8]=150;map[0,9]=50;map[0,10]=0
    map[0,11]=10; map[0,12]=10;map[0,13]=20;map[0,14]=30;map[0,15]=50;map[0,16]=40;map[0,17]=20;map[0,18]=10;map[0,19]=10

    map[1,0]=0; map[1,1]=0;map[1,1]=10;map[1,3]=20;map[1,4]=40;map[1,5]=60;map[1,6]=100;map[1,7]=210;map[1,8]=160;map[1,9]=50;map[1,10]=10
    map[1,11]=10; map[1,12]=10;map[1,13]=30;map[1,14]=30;map[1,15]=50;map[1,16]=40;map[1,17]=15;map[1,18]=10;map[1,19]=10

    map[2,0]=0; map[2,1]=0;map[2,1]=10;map[2,3]=20;map[2,4]=40;map[2,5]=60;map[2,6]=100;map[2,7]=220;map[2,8]=190;map[2,9]=120;map[2,10]=80
    map[2,11]=70; map[2,12]=30;map[2,13]=60;map[2,14]=50;map[2,15]=90;map[2,16]=60;map[2,17]=15;map[2,18]=10;map[2,19]=10

    map[3,0]=0; map[3,1]=0;map[3,1]=10;map[3,3]=30;map[3,4]=40;map[3,5]=70;map[3,6]=110;map[3,7]=240;map[3,8]=210;map[3,9]=150;map[3,10]=120
    map[3,11]=80; map[3,12]=50;map[3,13]=60;map[3,14]=50;map[3,15]=90;map[3,16]=60;map[3,17]=15;map[3,18]=10;map[3,19]=10

    map[4,0]=0; map[4,1]=0;map[4,1]=15;map[4,3]=40;map[4,4]=50;map[4,5]=100;map[4,6]=150;map[4,7]=250;map[4,8]=230;map[4,9]=180;map[4,10]=130
    map[4,11]=100; map[4,12]=80;map[4,13]=70;map[4,14]=50;map[4,15]=50;map[4,16]=50;map[4,17]=15;map[4,18]=10;map[4,19]=10

    map[5,0]=0; map[5,1]=0;map[5,1]=10;map[5,3]=30;map[5,4]=40;map[5,5]=70;map[5,6]=110;map[5,7]=240;map[5,8]=210;map[5,9]=150;map[5,10]=120
    map[5,11]=80; map[5,12]=50;map[5,13]=60;map[5,14]=50;map[5,15]=90;map[5,16]=60;map[5,17]=15;map[5,18]=10;map[5,19]=10

    map[6,0]=0; map[6,1]=0;map[6,1]=10;map[6,3]=10;map[6,4]=15;map[6,5]=40;map[6,6]=80;map[6,7]=150;map[6,8]=140;map[6,9]=120;map[6,10]=100
    map[6,11]=60; map[6,12]=30;map[6,13]=30;map[6,14]=20;map[6,15]=30;map[6,16]=15;map[6,17]=10;map[6,18]=0;map[6,19]=0

    map[7,0]=0; map[7,1]=0;map[7,1]=10;map[7,3]=10;map[7,4]=10;map[7,5]=20;map[7,6]=30;map[7,7]=80;map[7,8]=70;map[7,9]=60;map[7,10]=50
    map[7,11]=40; map[7,12]=20;map[7,13]=10;map[7,14]=10;map[7,15]=0;map[7,16]=10;map[7,17]=10;map[7,18]=0;map[7,19]=0

    map[8,0]=0; map[8,1]=0;map[8,1]=10;map[8,3]=10;map[8,4]=10;map[8,5]=15;map[8,6]=20;map[8,7]=40;map[8,8]=30;map[8,9]=20;map[8,10]=20
    map[8,11]=20; map[8,12]=20;map[8,13]=10;map[8,14]=10;map[8,15]=5;map[8,16]=5;map[8,17]=0;map[8,18]=0;map[8,19]=0

    map[9,0]=0; map[9,1]=0;map[9,1]=10;map[9,3]=10;map[9,4]=10;map[9,5]=15;map[9,6]=20;map[9,7]=40;map[9,8]=30;map[9,9]=20;map[9,10]=20
    map[9,11]=20; map[9,12]=20;map[9,13]=10;map[9,14]=10;map[9,15]=5;map[9,16]=5;map[9,17]=0;map[9,18]=0;map[9,19]=0

    map[10,0]=0; map[10,1]=0;map[10,1]=10;map[10,3]=10;map[10,4]=10;map[10,5]=15;map[10,6]=10;map[10,7]=15;map[10,8]=10;map[10,9]=12
    map[10,10]=8;map[10,11]=5; map[10,12]=5;map[10,13]=10;map[10,14]=0;map[10,15]=0;map[10,16]=0;map[10,17]=0;map[10,18]=0;map[10,19]=0

    map[11,0]=0; map[11,1]=0;map[11,1]=10;map[11,3]=10;map[11,4]=10;map[11,5]=15;map[11,6]=10;map[11,7]=15;map[11,8]=10;map[11,9]=12
    map[11,10]=8;map[11,11]=5; map[11,12]=5;map[11,13]=10;map[11,14]=0;map[11,15]=0;map[11,16]=0;map[11,17]=0;map[11,18]=0;map[11,19]=0

    map[12,0]=0; map[12,1]=10;map[12,1]=10;map[12,3]=20;map[12,4]=20;map[12,5]=40;map[12,6]=30;map[12,7]=40;map[12,8]=40;map[12,9]=50
    map[12,10]=20;map[12,11]=30; map[12,12]=40;map[12,13]=40;map[12,14]=20;map[12,15]=30;map[12,16]=10;map[12,17]=10;map[12,18]=5;map[12,19]=0

    map[13,0]=0; map[13,1]=10;map[13,1]=15;map[13,3]=30;map[13,4]=40;map[13,5]=70;map[13,6]=60;map[13,7]=80;map[13,8]=80;map[13,9]=100
    map[13,10]=50;map[13,11]=80; map[13,12]=60;map[13,13]=50;map[13,14]=30;map[13,15]=20;map[13,16]=10;map[13,17]=10;map[13,18]=5;map[13,19]=0

    map[14,0]=0; map[14,1]=10;map[14,1]=20;map[14,3]=40;map[14,4]=60;map[14,5]=90;map[14,6]=90;map[14,7]=100;map[14,8]=110;map[14,9]=130
    map[14,10]=90;map[14,11]=90; map[14,12]=70;map[14,13]=60;map[14,14]=50;map[14,15]=20;map[14,16]=10;map[14,17]=10;map[14,18]=5;map[14,19]=0

    map[15,0]=0; map[15,1]=20;map[15,1]=30;map[15,3]=50;map[15,4]=80;map[15,5]=100;map[15,6]=120;map[15,7]=150;map[15,8]=170;map[15,9]=200
    map[15,10]=130;map[15,11]=100; map[15,12]=90;map[15,13]=70;map[15,14]=50;map[15,15]=20;map[15,16]=10;map[15,17]=10;map[15,18]=5;map[15,19]=0

    map[16,0]=10; map[16,1]=15;map[16,1]=40;map[16,3]=60;map[16,4]=100;map[16,5]=120;map[16,6]=140;map[16,7]=170;map[16,8]=190;map[16,9]=220
    map[16,10]=180;map[16,11]=150; map[16,12]=120;map[16,13]=100;map[16,14]=80;map[16,15]=50;map[16,16]=30;map[16,17]=30;map[16,18]=10;map[16,19]=0

    map[17,0]=0; map[17,1]=5;map[17,1]=20;map[17,3]=40;map[17,4]=70;map[17,5]=80;map[17,6]=100;map[17,7]=130;map[17,8]=140;map[17,9]=170
    map[17,10]=140;map[17,11]=120; map[17,12]=100;map[17,13]=80;map[17,14]=60;map[17,15]=30;map[17,16]=10;map[17,17]=10;map[17,18]=0;map[17,19]=0

    map[18,0]=0; map[18,1]=5;map[18,1]=15;map[18,3]=20;map[18,4]=50;map[18,5]=50;map[18,6]=80;map[18,7]=100;map[18,8]=100;map[18,9]=130
    map[18,10]=130;map[18,11]=110; map[18,12]=80;map[18,13]=60;map[18,14]=40;map[18,15]=30;map[18,16]=10;map[18,17]=0;map[18,18]=0;map[18,19]=0

    map[19,0]=0; map[19,1]=5;map[19,1]=10;map[19,3]=10;map[19,4]=20;map[19,5]=30;map[19,6]=40;map[19,7]=60;map[19,8]=50;map[19,9]=10
    map[19,10]=10;map[19,11]=80; map[19,12]=50;map[19,13]=30;map[19,14]=20;map[19,15]=10;map[19,16]=10;map[19,17]=0;map[19,18]=0;map[19,19]=0




def generate_map(num_obstacles,dim):
    map = np.zeros((dim,dim))
    
    #add_steepness(map)

    for i in range(num_obstacles):
        center = rnd.randint(0+(dim/10),dim-(dim/10),size=2)
        radius = rnd.randint(dim/50,dim/10)
        Y, X = np.ogrid[:dim, :dim]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)
        mask = (dist_from_center <= radius)
        map[mask] = 255

    return map
from matplotlib.lines import Line2D
import numpy.random as rnd
import numpy as np 
import matplotlib.pyplot as plt
from math import sqrt
import PathVisualization as PV

# function to check if the points in a path are between the bounds
def checkBounds(bounds,path):
    for i in range(len(bounds)):
        if path[i][0]<bounds[i][0]: #if particle is less than lower bound
            path[i][0]=bounds[i][0]
        elif path[i][0]>bounds[i][1]: #if particle is more than higher bound
            path[i][0]=bounds[i][1]

        if path[i][1]<bounds[i][0]: #if particle is less than lower bound
            path[i][1]=bounds[i][0]
        elif path[i][1]>bounds[i][1]: #if particle is more than higher bound
            path[i][1]=bounds[i][1]
    

# function to check if a segment crosses an obstacle
def checkSegment(start, end):
    found_obstacle = False
    current_point = np.copy(start)

    x_distance = end[0]-start[0]
    y_distance = end[1]-start[1]
    if y_distance == 0:
        ratio = 10000
    else:
        ratio = abs(float(x_distance)/float(y_distance))

    while not np.array_equal(current_point,end) and not found_obstacle:
        x_distance = end[0]-current_point[0]
        y_distance = end[1]-current_point[1]
        current_point = PV.next_point(current_point,x_distance,y_distance,ratio)
        if map[current_point[0],current_point[1]] == 255:
            found_obstacle = True
        
    return not found_obstacle

# function to check if a path crosses an obstacle
def checkPath(path):
    
    if not checkSegment(np.array([0,0]),path[1]):
        return False
    if not checkSegment(path[-1],np.array([(map_dimension-1),(map_dimension-1)])):
        return False
    
    for i in range(len(path)-1):
        if not checkSegment(path[i],path[i+1]):
            return False

# function to get a random point between the bounds
def getRandomInt(bounds):
    return (np.round(rnd.uniform(bounds[0],bounds[1],2))).astype(int)

# function that initializes the variables of the problem
# f = fitting function
# bounds = bounds of the search space
# n = number of particles
# map = obstcales map of the environment
# map_dim = length of the map border
# num_points = number of points forming a path
def initialization(f,bounds,n, map, map_dim, num_points):
    paths = np.zeros((n,num_points))
    paths = paths.tolist()
    particle_val = paths[:]
    velocity = paths[:]
    local_best = paths[:]
    local_best_val = paths[:]
    
    global_best = []
    global_best_val = map_dim * num_points
    for p in range(n):  # generate random points for a path
        reset = True

        while reset:
            reset = False

            paths[p] = [getRandomInt(bounds[i]) for i in range(num_points)]
            paths[p] = [i.astype(int) for i in paths[p]]

            # check if the path crosses an obstacle and generate new random points
            while (not checkSegment(np.array([0,0]),paths[p][0]) or map[paths[p][0][0],paths[p][0][1]] == 255):
                paths[p][0] = getRandomInt(bounds[0])

            i = 1
            while i < num_points-1 and not reset:
                num_tries = 0
                while (not checkSegment(paths[p][i-1],paths[p][i]) or map[paths[p][i][0],paths[p][i][1]] == 255) and num_tries<20:
                    num_tries += 1
                    paths[p][i] = getRandomInt(bounds[i])
                
                if num_tries >= 20:
                    reset = True
                
                i += 1

            if not reset:
                num_tries = 0
                while (not checkSegment(paths[p][-1],np.array([map_dimension-1,map_dimension-1])) or \
                    not checkSegment(paths[p][-2],paths[p][-1]) or map[paths[p][-1][0],paths[p][-1][1]] == 255) and num_tries<20:
                    num_tries += 1
                    paths[p][-1] = getRandomInt(bounds[-1])
                
                if num_tries >= 20:
                    reset = True


        particle_val[p] = f(paths[p])
        velocity[p] = [rnd.uniform(-abs(bounds[i][1]-bounds[i][0]),abs(bounds[i][1]-bounds[i][0]),2) for i in range(num_points)]
        local_best[p] = paths[p]
        local_best_val[p] = particle_val[p]
        if(local_best_val[p] < global_best_val):
            global_best = local_best[p]
            global_best_val = f(global_best)


    return np.array(paths), np.array(particle_val), np.array(velocity), np.array(local_best), global_best, np.array(local_best_val)

# function that implements the PSO algorithm
# f = fitting function
# bounds = bounds of the search space
# phip,phig,omega = pso parameters
# n = number of particles
# map = obstcales map of the environment
# tol = precision of the pso algorithm
# num_iterations = max number of iterations
# map_dim = length of the map border
# num_points = number of points forming a path
def particle_swarm_optimization(f,bounds,n, phip, phig, omega_min, omega_max, tol, map, num_iterations, map_dim, num_points):

    paths, particle_val, velocity, local_best, global_best, local_best_val = initialization(f, bounds, n, map, map_dim, num_points)

    old_global_best_val = map_dim * num_points
    global_best_val = f(global_best)
    iteration = 1

    iterations_no_improv = 1
    found_new_global_best = False

    plt.ion()
    fig, ax = plt.subplots()
    ax.matshow(map)
    l = list()
    for i in range(n):
        l.append(ax.plot([],[])[0])
    
    # loop for pso iterations
    while abs(global_best_val - old_global_best_val) > tol  and iteration < num_iterations and iterations_no_improv < 50 :

        print("iteration num "+str(iteration)+"      current best path length = "+str(global_best_val))

        found_new_global_best = False

        # every 30 iterations and if the global best doesn't improve for some iterations we assign random velocity values
        if iteration%30 == 0 or iterations_no_improv%14 == 0:
            for j in range(n):
                velocity[j] = [rnd.uniform(-abs(bounds[i][1]-bounds[i][0]),abs(bounds[i][1]-bounds[i][0]),2) for i in range(len(bounds))]

        # loop to update every particle/path
        for p in range(n):
            reset = True

            while reset:
                reset = False

                # adaptive inertia weight to improve exploration and exploitation
                omega = omega_max - (((omega_max-omega_min)*i)/num_iterations)

                # compute the new particle velocity and position
                rp = rnd.uniform(0,1)
                rg = rnd.uniform(0,1)
                velocity[p] = np.add(np.add([i*omega for i in velocity[p]] \
                ,[phip*rp*i for i in ([x1-x2 for (x1,x2) in zip(local_best[p],paths[p])])]) \
                ,[phig*rg*i for i in ([x1-x2 for (x1,x2) in zip(global_best,paths[p])])])

                #if particle_velocity[i].any() > vmax : #is any velocity is greater than vmax
                #        particle_velocity[i,:]=vmax #set velocity to vmax

                paths[p] = (np.round(np.add(paths[p],velocity[p]))).astype(int)

                checkBounds(bounds,paths[p])

                # check if the path crosses an obstacle and generate new random points
                while (not checkSegment(np.array([0,0]),paths[p][0]) or map[paths[p][0][0],paths[p][0][1]] == 255):
                    paths[p][0] = getRandomInt(bounds[0])

                i = 1
                while i < num_points-1 and not reset:
                    num_tries = 0
                    while (not checkSegment(paths[p][i-1],paths[p][i]) or map[paths[p][i][0],paths[p][i][1]] == 255) and num_tries<20:
                        num_tries += 1
                        paths[p][i] = getRandomInt(bounds[i])
                    
                    if num_tries >= 20:
                        reset = True

                    i += 1

                if not reset:
                    num_tries = 0
                    while (not checkSegment(paths[p][-1],np.array([map_dimension-1,map_dimension-1])) or \
                        not checkSegment(paths[p][-2],paths[p][-1]) or map[paths[p][-1][0],paths[p][-1][1]] == 255) and num_tries<20:
                        num_tries += 1
                        paths[p][-1] = getRandomInt(bounds[-1])
                    
                    if num_tries >= 20:
                        reset = True

                
            particle_val[p] = f(paths[p])

            # update local and global bests
            if particle_val[p] < local_best_val[p]:
                local_best[p] = paths[p]
                local_best_val[p] = f(local_best[p])
            
            if local_best_val[p] < global_best_val:
                old_global_best_val = global_best_val
                global_best = local_best[p]
                global_best_val = local_best_val[p]
                found_new_global_best = True

        
        iteration += 1
        
        if found_new_global_best:
            iterations_no_improv = 1
        else:
            iterations_no_improv += 1

        # plot the paths
        for i in range(n):
            path = np.array(local_best[i])
            x,y = path.T
            x = np.append(np.array([0]),np.append(x,map_dimension-1))
            y = np.append(np.array([0]),np.append(y,map_dimension-1))
            l[i].set_xdata(y)
            l[i].set_ydata(x)
        
        plt.pause(0.1)
        plt.draw()

    return global_best, global_best_val



def euclidean_distance(x):
    total_distance = 0
    total_distance += sqrt((x[0][0])**2 + (x[0][1])**2)
    total_distance += sqrt(((map_dimension-1)-x[-1][0])**2 + ((map_dimension-1)-x[-1][1])**2)
    for i in range(len(x)-1):
        total_distance += sqrt((x[i][0]-x[i+1][0])**2 + (x[i][1]-x[i+1][1])**2)
    
    return total_distance

def manhattan_distance(x):
    total_distance = 0
    total_distance += x[0][0]+x[0][1]
    total_distance += (map_dimension-x[-1][0]) + (map_dimension-x[-1][1])
    for i in range(len(x)-1):
        total_distance += abs(x[i+1][0]-x[i][0]) + abs(x[i+1][1]-x[i][1])

    return total_distance


# problem parameters
map_dimension = 20
num_obstacles = 5
map = PV.generate_map(num_obstacles,map_dimension)
starting_point = (0,0)
goal = (map_dimension-1,map_dimension-1)
checkpoint_num = 3
f = euclidean_distance
dimension_bounds=[0,map_dimension-1]
bounds=[0]*checkpoint_num
for i in range(checkpoint_num):
    bounds[i]=dimension_bounds
p=10 # num of particles/paths
num_iterations = 500

#PSO parameters
phip = 1.1
phig = 6.2 
#omega = 0.7
omega_min = 0.7
omega_max = 1.1

tol=0.0000001



global_best, global_best_val = particle_swarm_optimization(f, bounds, p, phip, phig, omega_min, omega_max, tol, map, num_iterations, map_dimension, checkpoint_num)

print("global best path = "+str(global_best))
print("best path length = "+str(global_best_val))
#PV.drawPath(global_best,map,map_dimension)
PV.showPath(global_best,map,map_dimension)
plt.show(block=True)
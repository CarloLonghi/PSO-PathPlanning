import numpy as np 
import matplotlib.pyplot as plt
import numpy.random as rnd
import imageio


# function to plot a path
def show_path(path,map,dim,colormap):
    path = np.array(path)
    x,y = path.T
    x = np.append(np.array([0]),np.append(x,dim-1))
    y = np.append(np.array([0]),np.append(y,dim-1))
    
    plt.matshow(map,cmap=colormap)
    plt.title('Shortest path')
    plt.plot(y,x)

# function to generate random maps with obstacles
def generate_map(num_obstacles,dim):
    map = np.zeros((dim,dim))

    Y, X = np.ogrid[:dim, :dim]
    dist_from_center = np.sqrt((X - dim/2)**2 + (Y-dim/2)**2)
    mask = (dist_from_center <= 2)
    map[mask] = 255

    for i in range(num_obstacles-1):
        center = rnd.randint(0+(dim/5),dim-(dim/5),size=2)
        radius = rnd.randint(dim/10,dim/5)
        Y, X = np.ogrid[:dim, :dim]
        dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)
        mask = (dist_from_center <= radius)
        map[mask] = 255

    return map

# function to create a gif of the optimization process
def create_gif(iteration):
    images = []
    for i in range(1,iteration):
        images.append(imageio.imread('./images/animation/iteration'+str(i)+'.png'))
    imageio.mimsave('./images/anim.gif', images)
#!/usr/bin/env python
# coding: utf-8

# In[2]:


import numpy as np


# # FK using DH parameters

# In[3]:


def T(d, theta, r, alpha):
    cost = np.cos(theta)
    sint = np.sin(theta)
    cosa = np.cos(alpha)
    sina = np.sin(alpha)
    return np.array([[cost, -sint*cosa,  sint*sina, r*cost], 
                     [sint,  cost*cosa, -cost*sina, r*sint],
                     [   0,       sina,       cosa,      d],
                     [   0,          0,          0,      1]])

def pos(T):
    return np.array([T[0,3], T[1,3], T[2, 3]])

def getRotationMatrix(ax, angle):
    R = np.diag(np.ones(3))
    R[0,0] = R[1,1] = np.cos(angle)
    R[1,0] = -np.sin(angle)
    R[0,1] = -R[1,0]
    if ax != 2:
        R[:,[2, ax]] = R[:,[ax, 2]]
        R[[2, ax],:] = R[[ax, 2],:]
        if ax == 0:
            R = R.T
    return R


# In[4]:


# Shoulder: base revolves
# We expect a continuous height of 0.089159 mm and revolutions with a radius of 0.134 mm
theta_base = np.linspace(0, 2*np.pi, 9)
for theta in theta_base:
    Shoulder = T(0.089159, theta, 0.134, np.pi/2)
    print(np.round(pos(Shoulder), 3))


# In[5]:


# Elbow: base revolves
# Elbow is 90 degrees rotated when up: we expect a height of 0.089 mm + 0.425 mm = 0.514 mm

# When the elbow is at 0 degrees, we excpect the arm to extend 0.425 mm and 0.134 mm in two directions.
# This seems not to be the case, and we need a rotation about the Z axis.

theta_base = np.linspace(0,2*np.pi, 9)
for theta in theta_base:
    Shoulder = T(0.089159, theta, 0.134, np.pi/2)
    Shoulder[0,3], Shoulder[1,3] = -Shoulder[1,3], Shoulder[0,3]
    Elbow    = T(0, np.pi/2, 0.425, 0)
    Elbow    = Shoulder @ Elbow
    print(np.round(pos(Elbow), 3))


# In[6]:


# Elbow: shoulder revolves
# We expect the shoulder to be stationary at (0, 0.134, 0.089) mm and the elbow to get to negative z values,
# with a minimum at 0.089 mm - 0.425 mm = -0.336 mm while only changing X values. This appears to be correct.

theta_shoulder = np.linspace(0,2*np.pi, 9)
for theta in theta_shoulder:
    Shoulder = T(0.089159, 0, 0.134, np.pi/2)
    Shoulder[0,3], Shoulder[1,3] = -Shoulder[1,3], Shoulder[0,3]
    #print(np.round(pos(Shoulder), 3))
    Elbow    = T(0, theta, 0.425, 0)
    Elbow    = Shoulder @ Elbow
    print(np.round(pos(Elbow), 3))


# In[7]:


# Elbow2: base revolves
# Elbow2 is right next to the Elbow, and should be found at always the same height value as Elbow.
# We expect here a revolution with a radius of 0.134 mm - 0.119 mm = 0.015mm at a height of 0.514 mm.

# When the Elbow is down (0 degrees) we do get results as expected. 

theta_base = np.linspace(0,2*np.pi, 9)
for theta in theta_base:
    Shoulder = T(0.089159, theta, 0.134, np.pi/2)
    Shoulder[0,3], Shoulder[1,3] = -Shoulder[1,3], Shoulder[0,3]
    Elbow    = T(0, np.pi/2, 0.425, 0)
    Elbow    = Shoulder @ Elbow
    Elbow2   = T(0.119, 0, 0, 0)
    Elbow2   = Elbow @ Elbow2
    print(np.round(pos(Elbow2), 3))


# In[8]:


# Elbow2: shoulder revolves
# We expect the same as for the Elbow, though now a smaller distance from the origin away. This seems correct.

theta_shoulder = np.linspace(0,2*np.pi, 9)
for theta in theta_shoulder:
    Shoulder = T(0.089159, 0, 0.134, np.pi/2)
    Shoulder[0,3], Shoulder[1,3] = -Shoulder[1,3], Shoulder[0,3]
    #print(np.round(pos(Shoulder), 3))
    Elbow    = T(0, theta, 0.425, 0)
    Elbow    = Shoulder @ Elbow
    #print(np.round(pos(Elbow), 3))
    Elbow2   = T(0.119, 0, 0, 0)
    Elbow2   = Elbow @ Elbow2
    print(np.round(pos(Elbow2), 3))


# In[9]:


# Wrist1: base revolves
# We expect the total height of the wrist to be at 0.089 mm + 0.425 mm + 0.392 mm = 0.906 mm when all arms are extended.
# If the shoulder is rotated at 90 we expect to find the original height of 0.514 mm and an extension of 0.392 mm in one direction.
# If the shoulder is rotated at 0 degrees and the Elbow at -90 degrees, we expect a height of 0.089 mm + 0.392 mm = 0.481 mm 

theta_base = np.linspace(0,2*np.pi, 9)
for theta in theta_base:
    Shoulder = T(0.089159, theta, 0.134, np.pi/2)
    Shoulder[0,3], Shoulder[1,3] = -Shoulder[1,3], Shoulder[0,3]
    Elbow    = T(0, 0, 0.425, 0)
    Elbow    = Shoulder @ Elbow
    Elbow2   = T(0.119, np.pi/2, 0, 0)
    Elbow2   = Elbow @ Elbow2
    Wrist1   = T(0, 0, 0.39225, 0)
    Wrist1   = Elbow2 @ Wrist1
    print(np.round(pos(Wrist1), 3))
    


# In[10]:


# Wrist2:


# In[ ]:





# # Drawing the robot arm:

# In[15]:


from IPython.display import HTML
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 


# In[30]:


def getCurrentPos(theta_base, theta_shoulder, theta_elbow):
    Shoulder = T(0.089159, theta_base, 0.134, np.pi/2)
    Shoulder[0,3], Shoulder[1,3] = -Shoulder[1,3], Shoulder[0,3]
    
    Elbow    = T(0, -np.pi/2, -0.425, 0)
    # Elbow    = T(0, theta_shoulder, 0.425, 0)
    Elbow    = Shoulder @ Elbow
    
    Elbow2   = T(0.119, 0, 0, 0)
    # Elbow2   = T(0.119, theta_elbow, 0, 0)
    Elbow2   = Elbow @ Elbow2
    
    # Wrist1   = T(0, -np.pi/2, 0.39225, 0)
    Wrist1   = T(0, 0, -0.39225, 0)
    Wrist1   = Elbow2 @ Wrist1
    
    Wrist2   = T(0.09475, -np.pi/2, 0, np.pi/2)
    Wrist2   = Wrist1 @ Wrist2
    
    Wrist3   = T(0.09475, 0, 0, -np.pi/2)
    Wrist3   = Wrist2 @ Wrist3
    
    Tool   = T(0.0815, 0, 0, 0)
    Tool     = Wrist3 @ Tool
    
    ShoulderPos = pos(Shoulder)
    ElbowPos    = pos(Elbow)
    Elbow2Pos   = pos(Elbow2)
    Wrist1Pos   = pos(Wrist1)
    Wrist2Pos   = pos(Wrist2)
    Wrist3Pos   = pos(Wrist3)
    ToolPos     = pos(Tool)
    
    X = np.array([0,              0, ShoulderPos[0], ElbowPos[0], Elbow2Pos[0], Wrist1Pos[0], Wrist2Pos[0], Wrist3Pos[0], ToolPos[0]])
    Y = np.array([0,              0, ShoulderPos[1], ElbowPos[1], Elbow2Pos[1], Wrist1Pos[1], Wrist2Pos[1], Wrist3Pos[1], ToolPos[1]])
    Z = np.array([0, ShoulderPos[2], ShoulderPos[2], ElbowPos[2], Elbow2Pos[2], Wrist1Pos[2], Wrist2Pos[2], Wrist3Pos[2], ToolPos[2]])
    
    return X, Y, Z


def init():
    pass
    
def rotate_base(i):
    theta_base     = 2*np.pi/17 * i
    theta_shoulder = np.pi/2 + np.sin(np.pi/17 * i)
    theta_elbow    = -2*np.sin(np.pi/17 * i)
    
    currentPosX, currentPosY, currentPosZ = getCurrentPos(theta_base, theta_shoulder, theta_elbow)
    lines.set_data_3d(currentPosX, currentPosY, currentPosZ)
    balls._offsets3d = (currentPosX, currentPosY, currentPosZ)
    return (lines, balls,)


initPosX, initPosY, initPosZ = getCurrentPos(0, np.pi/2, 0)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlim3d(-0.5, 0.5)
ax.set_ylim3d(-0.5, 0.5)
ax.set_zlim3d(0, 1)

lines = ax.plot3D(initPosX, initPosY, initPosZ, 'black')[0]
balls = ax.scatter3D(initPosX, initPosY, initPosZ, c='r');

plt.close()

# call the animator. blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, rotate_base, frames=35, interval=100, blit=True)

HTML(anim.to_html5_video())


# In[497]:


# Set up formatting for the movie files
Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
anim.save('im.mp4', writer=writer)


# # Drawing examples:
# 

# In[471]:


# https://stackoverflow.com/questions/52630340/animate-a-3d-matrix-with-matplotlib-in-jupyter-notebook
    
from IPython.display import HTML
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
data = np.random.rand(100,50,50)

fig, ax = plt.subplots()

ax.set_xlim((0, 50))
ax.set_ylim((0, 50))

im = ax.imshow(data[0,:,:])

def init():
    im.set_data(data[0,:,:])
    return (im,)

# animation function. This is called sequentially
def animate(i):
    data_slice = data[i,:,:]
    im.set_data(data_slice)
    return (im,)

plt.close()

# call the animator. blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=5, interval=100, blit=True)

HTML(anim.to_html5_video())


# # Old computations:

# In[226]:


# Shoulder:
theta_shoulder = np.linspace(0,2*np.pi, 9)
for theta in theta_shoulder:
    Base     = T(0.089159, 0, 0, np.pi/2)
    Shoulder = T(0, theta, 0.134, 0)
    Elbow    = Base @ Shoulder
    print(np.round(pos(Elbow), 3))


# In[216]:


# Elbow:
theta_elbow = np.linspace(0,2*np.pi, 9)
for theta in theta_elbow:
    Base     = T(0.089159, theta, 0, np.pi/2)
    Shoulder = T(0, 0, 0.134, 0)
    Shoulder = Base @ Shoulder
    print(np.round(pos(Shoulder), 3))
    Elbow    = T(-0.425, 0, 0, 0)
    
    Elbow = Shoulder @ Elbow
    print(np.round(pos(Elbow), 3), "\n")


# # Using modified DH parameters:

# In[291]:


def T(d, theta, r, alpha):
    cost = np.cos(theta)
    sint = np.sin(theta)
    cosa = np.cos(alpha)
    sina = np.sin(alpha)
    return np.array([[     cost,      -sint,     0,       r], 
                     [sint*cosa,  cost*cosa, -sina, -d*sina],
                     [sint*sina,  cost*sina,  cosa,  d*cosa],
                     [        0,          0,     0,       1]])


def pos(T):
    return np.array([T[0,3], T[1,3], T[2, 3]])


# In[292]:


# Base:
theta_base = np.linspace(0, 2*np.pi, 9)
for theta in theta_base:
    Base = T(0.089159, theta, 0.134, np.pi/2)
    print(np.round(pos(Base), 3))


# # Using normal rotations:
# 

# In[282]:


def position(offset, theta, armlength):
    return np.array([armlength*np.cos(theta), armlength*np.sin(theta), offset])

def getRotationMatrix(ax, angle):
    R = np.diag(np.ones(3))
    R[0,0] = R[1,1] = np.cos(angle)
    R[1,0] = -np.sin(angle)
    R[0,1] = -R[1,0]
    if ax != 2:
        R[:,[2, ax]] = R[:,[ax, 2]]
        R[[2, ax],:] = R[[ax, 2],:]
        if ax == 0:
            R = R.T
    return R


# In[281]:


# Base gives Shoulder:
theta_base = np.linspace(0,2*np.pi, 9)
for theta in theta_base:
    Origin = np.zeros(3)
    Base   = position(0.089159, theta, 0.134)
    print(np.round(Base, 3))


# In[285]:


# Shoulder gives Elbow:
theta_base = np.linspace(0,2*np.pi, 9)
for theta in theta_base:
    Base     = position(0.089159, theta, 0.134)
    Shoulder = getRotationMatrix(1, -np.pi/2) @ position(0, 0, 0.425)
    print(np.round(Shoulder, 3))


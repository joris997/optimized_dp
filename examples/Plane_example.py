import imp
import numpy as np
import matplotlib.pyplot as plt
import math

# Utility functions to initialize the problem
from odp.Grid import Grid
from odp.Shapes import *

# Specify the  file that includes dynamic systems
from odp.dynamics import DubinsCapture
from odp.dynamics import DubinsCar4D2
from odp.dynamics import DubinsCar4D
from odp.dynamics import SAM
# Plot options
from odp.Plots import PlotOptions, plot_isosurface, plot_valuefunction, plot_target
# Solver core
from odp.solver import HJSolver, computeSpatDerivArray
# Sime core
from odp.compute_trajectory import compute_opt_traj


##################### MANUAL EXAMPLE #####################
gMin = np.array([-3,-3,-2,-math.pi])
gMax = np.array([3,3,2,math.pi])
gN = np.array([21,21,21,21])
g = Grid(gMin, gMax, 4, gN,[3])

##############
### TARGET ###
##############
# target = CylinderShape(g,[2,3],np.zeros(2),0.5)
target = ShapeRectangle(g, np.array([-0.5,-0.5,-0.2,-math.pi]), np.array([0.5,0.5,0.2,math.pi]))

#################
### OBSTACLES ###
#################
obs1 = CylinderShape(g,[2,3],np.array([0,-2]),0.5)
obs2 = CylinderShape(g,[2,3],np.array([0,2]),0.5)
# get the total obstacle which is the max value of the two obstacles
obstacles = np.minimum(obs1,obs2)


t0 = 0
tMax = 6
dt = 0.05
tau = np.arange(start=t0, stop=tMax, step=dt)

# robot = DubinsCar4D(x=[2.25,0.25,1.5,-math.pi],
#                     uMin=[-1,-1], uMax=[1,1],
#                     dMin=[-0.25,-0.25], dMax=[0.25,0.25],
#                     uMode="min", dMode="max")
robot = SAM(x=[2.25,0.25,1.5,-math.pi],
                    uMin=[-1,-1], uMax=[1,1],
                    dMin=[-0.25,-0.25], dMax=[0.25,0.25],
                    uMode="min", dMode="max")

po = PlotOptions(do_plot=True, plot_type="set", plotDims=[0,1,2],slicesCut=[0])

#################
### SOLVE HJI ###
#################
# In this example, we compute a Backward Reachable Tube
compMethods = {"TargetSetMode":   "minVWithV0",
               "ObstacleSetMode": "maxVWithObstacle"}
# HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
data = HJSolver(robot, g, [target,obstacles], tau, compMethods, po, saveAllTimeSteps=True )

#######################
### OPTIMAL CONTROL ###
#######################
traj, opt_u, opt_d, t = compute_opt_traj(robot, g, data, tau, robot.x)
# opt_ctrl = my_car.optCtrl_inPython(state_vector, spat_deriv_vector)
# print("Optimal control is {}\n".format(opt_ctrl))

# plot the trajectory [x,y,v,theta]
# 4 figures, top left is xy, top right is x and y over time, bottom left is theta over time, bottom right is v over time
# figure needs to be quite big
fig, axs = plt.subplots(2,2,figsize=(10,10))
axs[0, 0].plot(traj[:,0], traj[:,1])
plot_target(axs[0,0],g,target,[1,1,0,0],[0,0,0,0])
plot_target(axs[0,0],g,obstacles,[1,1,0,0],[0,0,0,0],color='r')
axs[0, 0].set_title('xy')

axs[0, 1].plot(tau, traj[:,0])
axs[0, 1].plot(tau, traj[:,1])
axs[0, 1].set_title('x and y over time')

axs[1, 0].plot(tau, traj[:,2])
axs[1, 0].set_title('theta over time')

axs[1, 1].plot(tau, traj[:,3])
axs[1, 1].set_title('v over time')

fig.savefig('figures/traj.png')
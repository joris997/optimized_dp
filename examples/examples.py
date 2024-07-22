import numpy as np
# Utility functions to initialize the problem
from odp.Grid import Grid
from odp.Shapes import *

# Specify the  file that includes dynamic systems
from odp.dynamics import DubinsCapture
from odp.dynamics import DubinsCar4D2
from odp.dynamics import DubinsCar4D
from odp.dynamics import SAM
from odp.dynamics import FreeFlyer
# Plot options
from odp.Plots import PlotOptions
# Solver core
from odp.solver import HJSolver, computeSpatDerivArray

import math

""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- Initialize plotting option
- Call HJSolver function
"""

# ##################################################### EXAMPLE 1 #####################################################

# g = Grid(np.array([-4.0, -4.0, -math.pi]), np.array([4.0, 4.0, math.pi]), 3, np.array([40, 40, 40]), [2])

# # Implicit function is a spherical shape - check out CylinderShape API
# Initial_value_f = CylinderShape(g, [], np.zeros(3), 1)

# # Look-back length and time step of computation
# lookback_length = 2.0
# t_step = 0.05

# small_number = 1e-5
# tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# # Specify the dynamical object. DubinsCapture() has been declared in dynamics/
# # Control uMode is maximizing, meaning that we're avoiding the initial target set
# my_car = DubinsCapture(uMode="max", dMode="min")

# # Specify how to plot the isosurface of the value function ( for higher-than-3-dimension arrays, which slices, indices
# # we should plot if we plot at all )
# po2 = PlotOptions(do_plot=True, plot_type="set", plotDims=[0,1,2],
#                   slicesCut=[])

# """
# Assign one of the following strings to `TargetSetMode` to specify the characteristics of computation
# "TargetSetMode":
# {
# "none" -> compute Backward Reachable Set,
# "minVWithV0" -> min V with V0 (compute Backward Reachable Tube),
# "maxVWithV0" -> max V with V0,
# "maxVWithVInit" -> compute max V over time,
# "minVWithVInit" -> compute min V over time,
# "minVWithVTarget" -> min V with target set (if target set is different from initial V0)
# "maxVWithVTarget" -> max V with target set (if target set is different from initial V0)
# }

# (optional)
# Please specify this mode if you would like to add another target set, which can be an obstacle set
# for solving a reach-avoid problem
# "ObstacleSetMode":
# {
# "minVWithObstacle" -> min with obstacle set,
# "maxVWithObstacle" -> max with obstacle set
# }
# """

# # In this example, we compute a Backward Reachable Tube
# compMethods = { "TargetSetMode": "minVWithV0"}
# # HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
# result = HJSolver(my_car, g, Initial_value_f, tau, compMethods, po2, saveAllTimeSteps=True )

# last_time_step_result = result[..., 0]
# # Compute spatial derivatives at every state
# x_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=1, accuracy="low")
# y_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=2, accuracy="low")
# T_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=3, accuracy="low")

# # Let's compute optimal control at some random idices
# spat_deriv_vector = (x_derivative[10,20,30], y_derivative[10,20,30], T_derivative[10,20,30])
# state_vector = (g.grid_points[0][10], g.grid_points[1][20], g.grid_points[2][30])

# # Compute the optimal control
# opt_ctrl = my_car.optCtrl_inPython(state_vector, spat_deriv_vector)
# print("Optimal control is {}\n".format(opt_ctrl))

# ##################################################### EXAMPLE 2 #####################################################

# g = Grid(np.array([-3.0, -1.0, 0.0, -math.pi]), np.array([3.0, 4.0, 4.0, math.pi]), 4, np.array([60, 60, 20, 36]), [3])

# # Define my object
# my_car = DubinsCar4D2()

# # Use the grid to initialize initial value function
# Initial_value_f = CylinderShape(g, [2,3], np.zeros(4), 1)

# # Look-back length and time step
# lookback_length = 1.0
# t_step = 0.05

# small_number = 1e-5

# tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# po = PlotOptions(do_plot=True, plot_type="value", plotDims=[0,1],
#                   slicesCut=[19, 30])

# # In this example, we compute a Backward Reachable Tube
# compMethods = { "TargetSetMode": "minVWithV0"}
# result = HJSolver(my_car, g, Initial_value_f, tau, compMethods, po, saveAllTimeSteps=True)

# last_time_step_result = result[..., 0]

# # Compute spatial derivatives at every state
# x_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=1, accuracy="low")
# y_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=2, accuracy="low")
# v_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=3, accuracy="low")
# T_derivative = computeSpatDerivArray(g, last_time_step_result, deriv_dim=4, accuracy="low")

# # Let's compute optimal control at some random idices
# spat_deriv_vector = (x_derivative[10,20,15,15], y_derivative[10,20,15,15],
#                      v_derivative[10,20,15,15], T_derivative[10,20,15,15])

# # Compute the optimal control
# opt_a, opt_w = my_car.optCtrl_inPython(spat_deriv_vector)
# print("Optimal accel is {}\n".format(opt_a))
# print("Optimal rotation is {}\n".format(opt_w))

# ##################################################### EXAMPLE 3 #####################################################

# # Third scenario with Reach-Avoid set
# g = Grid(np.array([-4.0, -4.0, -math.pi]), np.array([4.0, 4.0, math.pi]), 3, np.array([40, 40, 40]), [2])

# # Reacha9ble set
# goal = CylinderShape(g, [2], np.zeros(3), 0.5)

# # Avoid set
# obstacle = CylinderShape(g, [2], np.array([1.0, 1.0, 0.0]), 0.5)

# # Look-back length and time step
# lookback_length = 1.5
# t_step = 0.05

# small_number = 1e-5
# tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# my_car = DubinsCapture(uMode="min", dMode="max")

# po2 = PlotOptions(do_plot=True, plot_type="set", plotDims=[0,1,2],
#                   slicesCut=[])

# """
# Assign one of the following strings to `TargetSetMode` to specify the characteristics of computation
# "TargetSetMode":
# {
# "none" -> compute Backward Reachable Set,
# "minVWithV0" -> min V with V0 (compute Backward Reachable Tube),
# "maxVWithV0" -> max V with V0,
# "maxVWithVInit" -> compute max V over time,
# "minVWithVInit" -> compute min V over time,
# "minVWithVTarget" -> min V with target set (if target set is different from initial V0)
# "maxVWithVTarget" -> max V with target set (if target set is different from initial V0)
# }

# (optional)
# Please specify this mode if you would like to add another target set, which can be an obstacle set
# for solving a reach-avoid problem
# "ObstacleSetMode":
# {
# "minVWithObstacle" -> min with obstacle set,
# "maxVWithObstacle" -> max with obstacle set
# }
# """

# compMethods = { "TargetSetMode": "none",
#                 "ObstacleSetMode": "maxVWithObstacle"}
# # HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
# result = HJSolver(my_car, g, [goal, obstacle], tau, compMethods, po2, saveAllTimeSteps=True )

# ##################################################### EXAMPLE SAM #####################################################

# def ShapeRectangleByCenter(g, center, width):
#     return ShapeRectangle(g, center - width/2, center + width/2)


# g = Grid(np.array([-1,-3,-5,-math.pi]), 
#          np.array([9,3,5,math.pi]), 4,
#          np.array([41,41,41,41]), [3])
# # goal = ShapeRectangleByCenter(g, np.array([7,-2,0,0]), np.array([2,2,2.02,math.pi]))
# goal = ShapeRectangleByCenter(g, np.array([4,2,0,0]), np.array([2,2,2.02,math.pi]))
# obs = CylinderShape(g,[2,3],np.array([4,0]),1.0)

# dMax = 1.1
# # robot = DubinsCar4D(uMin=np.array([-4,-1]),     uMax=np.array([4,1]),
# #                     dMin=np.array([-dMax,-dMax]), dMax=np.array([dMax,dMax]),
# #                     uMode="min", dMode="max")
# robot = SAM(uMin=np.array([-4,-1]),     uMax=np.array([4,1]),
#             dMin=np.array([-dMax,-dMax]), dMax=np.array([dMax,dMax]),
#             uMode="min", dMode="max")

# tau = np.arange(start=0, stop=5, step=0.05)

# po2 = PlotOptions(do_plot=True, plot_type="set", plotDims=[0,1,2], slicesCut=[0])

# compMethods = { "TargetSetMode":   "none"}#"minVWithV0"}
# # HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
# result = HJSolver(robot, g, goal, tau, compMethods, po2, saveAllTimeSteps=True )

##################################################### EXAMPLE FREEFLYER #####################################################

def ShapeRectangleByCenter(g, center, width):
    return ShapeRectangle(g, center - width/2, center + width/2)

g = Grid(np.array([-2,-2,-1,-1,-math.pi,-0.5]),
         np.array([2,2,1,1,math.pi,0.5]), 6,
         np.array([41,41,21,21,21,21]), [4])

goal = ShapeRectangleByCenter(g, np.array([0,0,0,0,0,0]), np.array([0.5,0.5,0.5,0.5,2*math.pi,1.0]))

dMax = 0.0
robot = FreeFlyer(uMin=np.array([-1,-1]),     uMax=np.array([1,1]),
                   dMin=np.array([-dMax,-dMax]), dMax=np.array([dMax,dMax]),
                   uMode="min", dMode="max")

tau = np.arange(start=0, stop=1, step=0.1)

po3 = PlotOptions(do_plot=False, plot_type="set", plotDims=[0,1], slicesCut=[0,0,0,0])

compMethods = { "TargetSetMode":   "none"}#"minVWithV0"}
result = HJSolver(robot, g, goal, tau, compMethods, po3, saveAllTimeSteps=False , verbose=True)

print("result.shape: ", result.shape)
import heterocl as hcl
import numpy as np

""" 2D Single Integrator DYNAMICS IMPLEMENTATION 
 dx = dx
 ddx = u_x + d_1
"""
class DoubleIntegrator1D:
    def __init__(self, x=[0,0], uMin = [-1.5], uMax = [1.5], 
                 dMin = [-0.25], dMax=[0.25],
                 uMode="min", dMode="max"):
        """Creates a 1D Double Integrator with the following states:
           X position, Y position
        Args:
            x (list, optional): Initial state . Defaults to [0,0,0,0].
            uMin (list, optional): Lowerbound of user control. Defaults to [-1,-1].
            uMax (list, optional): Upperbound of user control.
                                   Defaults to [1,1].
            dMin (list, optional): Lowerbound of disturbance to user control, . Defaults to [-0.25,-0.25].
            dMax (list, optional): Upperbound of disturbance to user control. Defaults to [0.25,0.25].
            uMode (str, optional): Accepts either "min" or "max".
                                   * "min" : have optimal control reach goal
                                   * "max" : have optimal control avoid goal
                                   Defaults to "min".
            dMode (str, optional): Accepts whether "min" or "max" and should be opposite of uMode.
                                   Defaults to "max".
        """
        self.mass = 15.0

        self.x = x
        self.uMax = uMax
        self.uMin = uMin
        self.dMax = dMax
        self.dMin = dMin
        assert(uMode in ["min", "max"])
        self.uMode = uMode
        if uMode == "min":
            assert(dMode == "max")
        else:
            assert(dMode == "min")
        self.dMode = dMode

    def opt_ctrl(self, t, state, spat_deriv):
        """
        :param t: time t
        :param state: tuple of coordinates
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return:
        """

        # Graph takes in 4 possible inputs, by default, for now
        opt_u_x = hcl.scalar(self.uMax[0], "opt_u_x")
        # Just create and pass back, even though they're not used
        in2   = hcl.scalar(0, "in3")

        if self.uMode == "max":
            with hcl.if_(spat_deriv[1] < 0):
                opt_u_x[0] = self.uMin[0]
        else:
            with hcl.if_(spat_deriv[1] > 0):
                opt_u_x[0] = self.uMin[0]
                
        # return 3, 4 even if you don't use them
        return (opt_u_x[0], in2[0])

    def opt_dstb(self, t, state, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        # Graph takes in 4 possible inputs, by default, for now
        opt_d_x = hcl.scalar(0, "opt_d_x")
        d2 = hcl.scalar(0, "d2")

        #with hcl.if_(self.dMode == "max"):
        if self.dMode == "max":
            with hcl.if_(spat_deriv[1] > 0):
                opt_d_x[0] = self.dMax[0]
            with hcl.elif_(spat_deriv[1] < 0):
                opt_d_x[0] = self.dMin[0]
        else:
            with hcl.if_(spat_deriv[1] > 0):
                opt_d_x[0] = self.dMin[0]
            with hcl.elif_(spat_deriv[1] < 0):
                opt_d_x[0] = self.dMax[0]

        return (opt_d_x[0], d2[0])

    def dynamics(self, t, state, uOpt, dOpt):
        x_dot = hcl.scalar(0, "x_dot")
        x_ddot = hcl.scalar(0, "x_ddot")

        x_dot[0] = state[1]
        x_ddot[0] = (uOpt[0] + dOpt[0])/self.mass

        return (x_dot[0], x_ddot[0])
    
    def optCtrl_inPython(self, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal control
        """
        opt_a = 0
        opt_w = 0
        if self.uMode == "max":
            if spat_deriv[1] >= 0:
                opt_a = self.uMax[0]
            elif spat_deriv[1] < 0:
                opt_a = self.uMin[0]
        else:
            if spat_deriv[1] >= 0:
                opt_a = self.uMin[0]
            elif spat_deriv[1] < 0:
                opt_a = self.uMax[0]
        return (opt_a)
    
    def optDstb_inPython(self, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        d1 = 0
        if self.dMode == "max":
            if spat_deriv[1] >= 0:
                d1 = self.dMax[0]
            elif spat_deriv[1] < 0:
                d1 = self.dMin[0]
        else:
            if spat_deriv[1] >= 0:
                d1 = self.dMin[0]
            elif spat_deriv[1] < 0:
                d1 = self.dMax[0]
        return (d1)

    def dynamics_python(self, state, uOpt, dOpt):
        """
        :param state: tuple of coordinates
        :param uOpt: tuple of optimal control
        :param dOpt: tuple of optimal disturbances
        :return: a tuple of state derivatives
        """
        x_dot = state[1]
        x_ddot = (uOpt[0] + dOpt[0])/self.mass
        return (x_dot, x_ddot)
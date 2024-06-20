import heterocl as hcl
import numpy as np

""" 2D Single Integrator DYNAMICS IMPLEMENTATION 
 x_dot = v_x + d_1
 y_dot = v_y + d_2
 """
class SingleIntegrator2D:
    def __init__(self, x=[0,0], uMin = [-1,-1], uMax = [1,1], 
                 dMin = [-0.25,-0.25], dMax=[0.25,0.25],
                 uMode="min", dMode="max"):
        """Creates a 2D Single Integrator with the following states:
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
        opt_u_y = hcl.scalar(self.uMax[1], "opt_u_y")
        # Just create and pass back, even though they're not used
        in3   = hcl.scalar(0, "in3")
        in4   = hcl.scalar(0, "in4")

        if self.uMode == "max":
            with hcl.if_(spat_deriv[0] < 0):
                opt_u_x[0] = self.uMin[0]
            with hcl.if_(spat_deriv[1] < 0):
                opt_u_y[0] = self.uMin[1]
        else:
            with hcl.if_(spat_deriv[0] > 0):
                opt_u_x[0] = self.uMin[0]
            with hcl.if_(spat_deriv[1] > 0):
                opt_u_y[0] = self.uMin[1]
                
        # return 3, 4 even if you don't use them
        return (opt_u_x[0] ,opt_u_y[0])

    def opt_dstb(self, t, state, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        # Graph takes in 4 possible inputs, by default, for now
        d1 = hcl.scalar(0, "d1")
        d2 = hcl.scalar(0, "d2")

        #with hcl.if_(self.dMode == "max"):
        if self.dMode == "max":
            with hcl.if_(spat_deriv[0] > 0):
                d1[0] = self.dMax[0]
            with hcl.elif_(spat_deriv[0] < 0):
                d1[0] = self.dMin[0]
            with hcl.if_(spat_deriv[1] > 0):
                d2[0] = self.dMax[1]
            with hcl.elif_(spat_deriv[1] < 0):
                d2[0] = self.dMin[1]
        else:
            with hcl.if_(spat_deriv[0] > 0):
                d1[0] = self.dMin[0]
            with hcl.elif_(spat_deriv[0] < 0):
                d1[0] = self.dMax[0]
            with hcl.if_(spat_deriv[1] > 0):
                d2[0] = self.dMin[1]
            with hcl.elif_(spat_deriv[1] < 0):
                d2[0] = self.dMax[1]

        return (d1[0], d2[0])

    def dynamics(self, t, state, uOpt, dOpt):
        x_dot = hcl.scalar(0, "x_dot")
        y_dot = hcl.scalar(0, "y_dot")

        x_dot[0] = uOpt[0] + dOpt[0]
        y_dot[0] = uOpt[1] + dOpt[1]

        return (x_dot[0], y_dot[0])
    
    def optCtrl_inPython(self, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal control
        """
        opt_a = 0
        opt_w = 0
        if self.uMode == "max":
            if spat_deriv[2] >= 0:
                opt_a = self.uMax[0]
            elif spat_deriv[2] < 0:
                opt_a = self.uMin[0]
            if spat_deriv[3] >= 0:
                opt_w = self.uMax[1]
            elif spat_deriv[3] < 0:
                opt_w = self.uMin[1]
        else:
            if spat_deriv[2] >= 0:
                opt_a = self.uMin[0]
            elif spat_deriv[2] < 0:
                opt_a = self.uMax[0]
            if spat_deriv[3] >= 0:
                opt_w = self.uMin[1]
            elif spat_deriv[3] < 0:
                opt_w = self.uMax[1]
        return (opt_a, opt_w)
    
    def optDstb_inPython(self, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        d1 = 0
        d2 = 0
        if self.dMode == "max":
            if spat_deriv[0] >= 0:
                d1 = self.dMax[0]
            elif spat_deriv[0] < 0:
                d1 = self.dMin[0]
            if spat_deriv[1] >= 0:
                d2 = self.dMax[1]
            elif spat_deriv[1] < 0:
                d2 = self.dMin[1]
        else:
            if spat_deriv[0] >= 0:
                d1 = self.dMin[0]
            elif spat_deriv[0] < 0:
                d1 = self.dMax[0]
            if spat_deriv[1] >= 0:
                d2 = self.dMin[1]
            elif spat_deriv[1] < 0:
                d2 = self.dMax[1]
        return (d1, d2)

    def dynamics_python(self, state, uOpt, dOpt):
        """
        :param state: tuple of coordinates
        :param uOpt: tuple of optimal control
        :param dOpt: tuple of optimal disturbances
        :return: a tuple of state derivatives
        """
        x_dot = uOpt[0] + dOpt[0]
        y_dot = uOpt[1] + dOpt[1]
        return (x_dot, y_dot)
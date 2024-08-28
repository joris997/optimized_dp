import heterocl as hcl
import numpy as np

""" 4D SAM DYNAMICS IMPLEMENTATION 
 x_dot = u_v * cos(theta) + d_1 * (A_front*cos^2(theta) + A_side*sin^2(theta))
 y_dot = u_v * sin(theta) + d_2 * (A_front*sin^2(theta) + A_side*cos^2(theta))
 theta_dot = u_theta
 """
class kinSAM:
    def __init__(self, x=[0,0,0], uMin = [-1,-1], uMax = [1,1], 
                 dMin = [-0.25,-0.25], dMax=[0.25,0.25], 
                 A_front=0.25, A_side=1.0, 
                 uMode="min", dMode="max"):
        """Creates a SAM with the following states:
           X position, Y position, speed, heading
           The controls are the acceleration and turn rate (angular speed)
           The disturbances are the noise in the velocity components.
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

        self.A_front = A_front
        self.A_side = A_side

    def opt_ctrl(self, t, state, spat_deriv):
        """
        :param t: time t
        :param state: tuple of coordinates
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return:
        """
        # System dynamics
        # x_dot = u_v * cos(theta) + d_1 * (A_front*cos^2(theta) + A_side*sin^2(theta))
        # y_dot = u_v * sin(theta) + d_2 * (A_front*sin^2(theta) + A_side*cos^2(theta))
        # theta_dot = u_theta

        # Graph takes in 4 possible inputs, by default, for now
        opt_a = hcl.scalar(self.uMax[0], "opt_a")
        opt_w = hcl.scalar(self.uMax[1], "opt_w")
        # Just create and pass back, even though they're not used
        in3   = hcl.scalar(0, "in3")

        # for ease of use
        parSum1 = hcl.scalar(0, "parSum1")
        parSum2 = hcl.scalar(0, "parSum2")
        parSum1[0] = spat_deriv[0] * hcl.cos(state[2]) + spat_deriv[1] * hcl.sin(state[2])
        parSum2[0] = spat_deriv[2]

        if self.uMode == "max":
            with hcl.if_(parSum1[0] < 0):
                opt_a[0] = self.uMin[0]
            with hcl.if_(parSum2[0] < 0):
                opt_w[0] = self.uMin[1]
        else:
            with hcl.if_(parSum1[0] > 0):
                opt_a[0] = self.uMin[0]
            with hcl.if_(parSum2[0] > 0):
                opt_w[0] = self.uMin[1]
                
        # return 3, 4 even if you don't use them
        return (opt_a[0] ,opt_w[0], in3[0])

    def opt_dstb(self, t, state, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        # Graph takes in 4 possible inputs, by default, for now
        d1 = hcl.scalar(self.dMin[0], "d1")
        d2 = hcl.scalar(self.dMin[1], "d2")
        # Just create and pass back, even though they're not used
        d3 = hcl.scalar(0., "d3")

        # for ease of use
        parSum1 = hcl.scalar(0, "parSum1")
        parSum2 = hcl.scalar(0, "parSum2")
        # parSum1[0] = spat_deriv[0] * (self.A_front * hcl.cos(state[2])*hcl.cos(state[2]) + self.A_side * hcl.sin(state[2])*hcl.sin(state[2]))
        # parSum2[0] = spat_deriv[1] * (self.A_front * hcl.sin(state[2])*hcl.sin(state[2]) + self.A_side * hcl.cos(state[2])*hcl.cos(state[2]))
        parSum1[0] = spat_deriv[0] * hcl.sqrt((self.A_front**2 * hcl.cos(state[2])*hcl.cos(state[2]) + self.A_side**2 * hcl.sin(state[2])*hcl.sin(state[2])))
        parSum2[0] = spat_deriv[1] * hcl.sqrt((self.A_front**2 * hcl.sin(state[2])*hcl.sin(state[2]) + self.A_side**2 * hcl.cos(state[2])*hcl.cos(state[2])))

        #with hcl.if_(self.dMode == "max"):
        if self.dMode == "max":
            with hcl.if_(parSum1[0] > 0):
                d1[0] = self.dMax[0]
            with hcl.if_(parSum2[0] > 0):
                d2[0] = self.dMax[1]
        else:
            with hcl.if_(parSum1[0] < 0):
                d1[0] = self.dMax[0]
            with hcl.if_(parSum2[0] < 0):
                d2[0] = self.dMax[1]

        return (d1[0], d2[0], d3[0])

    def dynamics(self, t, state, uOpt, dOpt):
        x_dot = hcl.scalar(0, "x_dot")
        y_dot = hcl.scalar(0, "y_dot")
        theta_dot = hcl.scalar(0, "theta_dot")

        x_dot[0] = uOpt[0] * hcl.cos(state[2]) + dOpt[0]*hcl.sqrt(self.A_front**2*(hcl.cos(state[2])*hcl.cos(state[2])) + self.A_side**2*(hcl.sin(state[2])*hcl.sin(state[2])))
        y_dot[0] = uOpt[0] * hcl.sin(state[2]) + dOpt[1]*hcl.sqrt(self.A_front**2*(hcl.sin(state[2])*hcl.sin(state[2])) + self.A_side**2*(hcl.cos(state[2])*hcl.cos(state[2])))
        theta_dot[0] = uOpt[1]

        return (x_dot[0], y_dot[0], theta_dot[0])
    
    def optCtrl_inPython(self, state, spat_deriv):
        # TODO: add state as input
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal control
        """
        opt_a = self.uMax[0]
        opt_w = self.uMax[1]

        parSum1 = spat_deriv[0] * np.cos(state[2]) + spat_deriv[1] * np.sin(state[2])
        parSum2 = spat_deriv[2]

        if self.uMode == "max":
            if parSum1 < 0:
                opt_a = self.uMin[0]
            if parSum2 < 0:
                opt_w = self.uMin[1]
        else:
            if parSum1 > 0:
                opt_a = self.uMin[0]
            if parSum2 > 0:
                opt_w = self.uMin[1]

        return (opt_a, opt_w)
            
    
    def optDstb_inPython(self, state, spat_deriv):
        # TODO: add state as input
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        d1 = self.dMin[0]
        d2 = self.dMin[1]
        if self.dMode == "max":
            if spat_deriv[0] >= 0:
                d1 = self.dMax[0]
            if spat_deriv[1] >= 0:
                d2 = self.dMax[1]
        else:
            if spat_deriv[0] < 0:
                d1 = self.dMax[0]
            if spat_deriv[1] < 0:
                d2 = self.dMax[1]
        return (d1, d2)

    def dynamics_inPython(self, state, uOpt, dOpt):
        """
        :param state: tuple of coordinates
        :param uOpt: tuple of optimal control
        :param dOpt: tuple of optimal disturbances
        :return: a tuple of state derivatives
        """
        x_dot = uOpt[0] * np.cos(state[2]) + dOpt[0]*(self.A_front*np.power(np.cos(state[2]),2) + self.A_side*np.power(np.sin(state[2]),2))
        y_dot = uOpt[0] * np.sin(state[2]) + dOpt[1]*(self.A_front*np.power(np.sin(state[2]),2) + self.A_side*np.power(np.cos(state[2]),2))
        theta_dot = uOpt[1]
        return (x_dot, y_dot, theta_dot)
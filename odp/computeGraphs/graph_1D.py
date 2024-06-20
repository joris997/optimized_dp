import heterocl as hcl
import numpy as np
from odp.computeGraphs.CustomGraphFunctions import *
from odp.spatialDerivatives.first_orderENO1D import *
from odp.spatialDerivatives.second_orderENO1D import *

#from user_definer import *
#def graph_1D(dynamics_obj, grid):
def graph_1D(my_object, g, compMethod, accuracy, generate_SpatDeriv=False, deriv_dim=1, verbose=False):
    V_f = hcl.placeholder(tuple(g.pts_each_dim), name="V_f", dtype=hcl.Float())
    V_init = hcl.placeholder(tuple(g.pts_each_dim), name="V_init", dtype=hcl.Float())
    l0 = hcl.placeholder(tuple(g.pts_each_dim), name="l0", dtype=hcl.Float())
    t = hcl.placeholder((2,), name="t", dtype=hcl.Float())
    # probe = hcl.placeholder(tuple(g.pts_each_dim), name="probe", dtype=hcl.Float())

    # Positions vector
    x1 = hcl.placeholder((g.pts_each_dim[0],), name="x1", dtype=hcl.Float())

    def graph_create(V_new, V_init, x1, t, l0):
        # Specify intermediate tensors
        deriv_diff1 = hcl.compute(V_init.shape, lambda *x: 0, "deriv_diff1")

        # Maximum derivative for each dim
        max_deriv1 = hcl.scalar(-1e9, "max_deriv1")

        # Min derivative for each dim
        min_deriv1 = hcl.scalar(1e9, "min_deriv1")

        # These variables are used to dissipation calculation
        max_alpha1 = hcl.scalar(-1e9, "max_alpha1")

        def step_bound():  # Function to calculate time step
            stepBoundInv = hcl.scalar(0, "stepBoundInv")
            stepBound = hcl.scalar(0, "stepBound")
            stepBoundInv[0] = max_alpha1[0] / g.dx[0] 
            stepBound[0] = 0.8 / stepBoundInv[0]
            with hcl.if_(stepBound > t[1] - t[0]):
                stepBound[0] = t[1] - t[0]
            t[0] = t[0] + stepBound[0]
            return stepBound[0]

            # Min with V_before
        def minVWithVInit(i):
            with hcl.if_(V_new[i] > V_init[i]):
                V_new[i] = V_init[i]

        def maxVWithVInit(i):
            with hcl.if_(V_new[i] < V_init[i]):
                V_new[i] = V_init[i]

        def maxVWithV0(i):  # Take the max
            with hcl.if_(V_new[i] < l0[i]):
                V_new[i] = l0[i]

        def minVWithV0(i):
            with hcl.if_(V_new[i] > l0[i]):
                V_new[i] = l0[i]

        # Calculate Hamiltonian for every grid point in V_init
        with hcl.Stage("Hamiltonian"):
            with hcl.for_(0, V_init.shape[0], name="i") as i:  # Plus 1 as for loop count stops at V_init.shape[0]
                # Variables to calculate dV_dx
                dV_dx_L = hcl.scalar(0, "dV_dx_L")
                dV_dx_R = hcl.scalar(0, "dV_dx_R")
                dV_dx = hcl.scalar(0, "dV_dx")

                # No tensor slice operation
                if accuracy == "low":
                    dV_dx_L[0], dV_dx_R[0] = spa_derivX(i, V_init, g)
                if accuracy == "medium":
                    dV_dx_L[0], dV_dx_R[0] = secondOrderX(i, V_init, g)

                # Saves spatial derivative diff into tables
                deriv_diff1[i] = dV_dx_R[0] - dV_dx_L[0]

                # Calculate average gradient
                dV_dx[0] = (dV_dx_L + dV_dx_R) / 2

                V_new[i] = dV_dx[0]*2

                # Use method of DubinsCar to solve optimal control instead
                uOpt = my_object.opt_ctrl(t, (x1[i],),
                                                (dV_dx[0],))
                dOpt = my_object.opt_dstb(t, (x1[i], ),
                                            (dV_dx[0], ))

                # Calculate dynamical rates of changes
                dx_dt = my_object.dynamics(t, (x1[i], ), uOpt, dOpt)

                # Calculate Hamiltonian terms:
                V_new[i] = -(dx_dt[0] * dV_dx[0])

                # Get derivMin
                with hcl.if_(dV_dx_L[0] < min_deriv1[0]):
                    min_deriv1[0] = dV_dx_L[0]
                with hcl.if_(dV_dx_R[0] < min_deriv1[0]):
                    min_deriv1[0] = dV_dx_R[0]

                # Get derivMax
                with hcl.if_(dV_dx_L[0] > max_deriv1[0]):
                    max_deriv1[0] = dV_dx_L[0]
                with hcl.if_(dV_dx_R[0] > max_deriv1[0]):
                    max_deriv1[0] = dV_dx_R[0]

        # Calculate the dissipation
        with hcl.Stage("Dissipation"):
            # Storing alphas
            dOptL1 = hcl.scalar(0, "dOptL1")
            # Find UPPER BOUND optimal disturbance
            dOptU1 = hcl.scalar(0, "dOptU1")
            alpha1 = hcl.scalar(0, "alpha1")

            # Lower bound optimal control
            uOptL1 = hcl.scalar(0, "uOptL1")

            # Find UPPER BOUND optimal disturbance
            uOptU1 = hcl.scalar(0, "uOptU1")

            with hcl.for_(0, V_init.shape[0], name="i") as i:
                dx_LL1 = hcl.scalar(0, "dx_LL1")

                dx_UL1 = hcl.scalar(0, "dx_UL1")

                dx_UU1 = hcl.scalar(0, "dx_UU1")

                dx_LU1 = hcl.scalar(0, "dx_LU1")

                # Find LOWER BOUND optimal disturbance
                dOptL1[0] = my_object.opt_dstb(t,  (x1[i], ),\
                                                                (min_deriv1[0], ))[0]

                dOptU1[0] = my_object.opt_dstb(t, (x1[i], ),\
                                                                (max_deriv1[0], ))[0]

                # Find LOWER BOUND optimal control
                uOptL1[0] = my_object.opt_ctrl(t, (x1[i], ), \
                                                                (min_deriv1[0], ))[0]

                    # Find UPPER BOUND optimal control
                uOptU1[0] = my_object.opt_ctrl(t, (x1[i], ),
                                                                (max_deriv1[0], ))[0]
                    # Find magnitude of rates of changes
                dx_LL1[0] = my_object.dynamics(t, (x1[i], ),
                                                                (uOptL1[0], ), \
                                                                (dOptL1[0], ))[0]
                dx_LL1[0] = my_abs(dx_LL1[0])

                dx_LU1[0] = my_object.dynamics(t, (x1[i], ),
                                                                (uOptL1[0], ), \
                                                                (dOptU1[0], ))[0]
                dx_LU1[0] = my_abs(dx_LU1[0])

                # Calculate alpha
                alpha1[0] = my_max(dx_LL1[0], dx_LU1[0])

                dx_UL1[0] = my_object.dynamics(t, (x1[i], ),\
                                                                (uOptU1[0], ), \
                                                                (dOptL1[0], ))[0]
                dx_UL1[0] = my_abs(dx_UL1[0])

                # Calculate alpha
                alpha1[0] = my_max(alpha1[0], dx_UL1[0])

                dx_UU1[0] = my_object.dynamics(t, (x1[i], ),
                                                                (uOptU1[0], ),\
                                                                (dOptU1[0], ))[0]
                dx_UU1[0] = my_abs(dx_UU1[0])
                # Calculate alpha
                alpha1[0] = my_max(alpha1[0], dx_UU1[0])

                diss = hcl.scalar(0, "diss")
                diss[0] = 0.5 * (
                        deriv_diff1[i] * alpha1[0])

                # Finally
                V_new[i] = -(V_new[i] - diss[0])

                # Calculate alphas
                with hcl.if_(alpha1[0] > max_alpha1[0]):
                    max_alpha1[0] = alpha1[0]



        # Determine time step
        delta_t = hcl.compute((1,), lambda x: step_bound(), name="delta_t")
        # Integrate
        result = hcl.update(V_new, lambda i: V_init[i] + V_new[i] * delta_t[0])

        # Different computation method check
        if compMethod == 'maxVWithV0' or compMethod == 'maxVWithVTarget':
            result = hcl.update(V_new, lambda i: maxVWithV0(i))
        if compMethod == 'minVWithV0' or compMethod == 'minVWithVTarget':
            result = hcl.update(V_new, lambda i: minVWithV0(i))
        if compMethod == 'minVWithVInit':
            result = hcl.update(V_new, lambda i: minVWithVInit(i))
        if compMethod == 'maxVWithVInit':
            result = hcl.update(V_new, lambda i: maxVWithVInit(i))

        # Copy V_new to V_init
        hcl.update(V_init, lambda i: V_new[i])
        return result

    def returnDerivative(V_array, Deriv_array):
        with hcl.Stage("ComputeDeriv"):
            with hcl.for_(0, V_array.shape[0], name="i") as i:
                dV_dx_L = hcl.scalar(0, "dV_dx_L")
                dV_dx_R = hcl.scalar(0, "dV_dx_R")
                if accuracy == "low":
                    if deriv_dim == 1:
                        dV_dx_L[0], dV_dx_R[0] = spa_derivX(i, V_array, g)
                if accuracy == "medium":
                    if deriv_dim == 1:
                        dV_dx_L[0], dV_dx_R[0] = secondOrderX(i, V_array, g)

                Deriv_array[i] = (dV_dx_L[0] + dV_dx_R[0]) / 2

    if generate_SpatDeriv == False:
        s = hcl.create_schedule([V_f, V_init, x1, t, l0], graph_create)
        ##################### CODE OPTIMIZATION HERE ###########################
        print("Optimizing\n") if verbose else None

        # Accessing the hamiltonian and dissipation stage
        s_H = graph_create.Hamiltonian
        s_D = graph_create.Dissipation

        # Thread parallelize hamiltonian and dissipation computation
        s[s_H].parallel(s_H.i)
        s[s_D].parallel(s_D.i)
    else:
        print("I'm here\n") if verbose else None
        s = hcl.create_schedule([V_init, V_f], returnDerivative)

    # Inspect IR
    # if args.llvm:
        #print(hcl.lower(s))

    # Return executable

    return (hcl.build(s))

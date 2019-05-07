#!/usr/bin/env python
#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2016 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.
#

# This is the main path of your do-mpc installation relative to the execution folder
path_do_mpc = '../'
# Add do-mpc path to the current directory
import sys
sys.path.insert(0,path_do_mpc+'code')
# Do not write bytecode to maintain clean directories
sys.dont_write_bytecode = True
# Compatibility for python 2.7 and python 3.0
from builtins import input
# Start CasADi
from casadi import *
# Import do-mpc core functionalities
import core_do_mpc
# Import do-mpc plotting and data managament functions
import data_do_mpc
import matplotlib.pyplot as plt
from move import move
from dist import dist
import math
import matplotlib.patches as patches
import numpy as np
"""
-----------------------------------------------
do-mpc: Definition of the do-mpc configuration
-----------------------------------------------
"""

# Import the user defined modules
import template_model
import template_optimizer
import template_observer
import template_simulator

class doMPC:
    #def __init__(self):
        

    def getOptControl(self,waypoint,obstacles,pose,twist):
        #weights
        w = [60,60,1,1e5]
        # Create the objects for each module
        model_1 = template_model.model(waypoint,obstacles,pose,twist,w)
        # Create an optimizer object based on the template and a model
        optimizer_1 = template_optimizer.optimizer(model_1)
        # Create an observer object based on the template and a model
        observer_1 = template_observer.observer(model_1)
        # Create a simulator object based on the template and a model
        simulator_1 = template_simulator.simulator(model_1)
        # Create a configuration
        configuration_1 = core_do_mpc.configuration(model_1, optimizer_1, observer_1, simulator_1)

        # Set up the solvers
        configuration_1.setup_solver()

        """
        ----------------------------
        do-mpc: MPC loop
        ----------------------------
        """
        x=pose
        Y_ref=waypoint
        lam=w[3]
        gamma=1

        #V=lam/((gamma+((xo[0]-x[0])**2)/(rx**2)+((xo[1]-x[1])**2)/(ry**2)))
        if len(obstacles.a):
            V = 0 
            for i in xrange(len(obstacles.a)):
                if obstacles.a[i]:
                    rx=obstacles.a[i]
                    ry=obstacles.b[i]
                    phi = atan(obstacles.m[i])
                    xo=obstacles.centroid[i]
                    

                    R = [[cos(x[2]), -sin(x[2])],[sin(x[2]),cos(x[2])]]

                    xo = np.matmul(R,xo) + pose[0:2]

                    V = V + (lam)/((gamma+(( (xo[0]-x[0])*cos(phi) + (xo[1]-x[1])*sin(phi) )**2)/(rx**2) + (( (xo[1]-x[0])*sin(phi) - (xo[1]-x[1])*cos(phi) )**2)/(ry**2) ))

                    
                    print "cost",i, V

            print "total cost", V

        print "x cost:",w[0]*(x[0]-Y_ref[0])**2,"y cost:",w[1]*(x[1]-Y_ref[1])**2,"theta cost:", w[2]*(x[2]-Y_ref[2])**2
        #a1=ax.scatter(x[0],x[1])
        #a2=ax.arrow(x[0],x[1],3*math.cos(x[2]),3*math.sin(x[2]), head_width=0.08, head_length=0.00002)
        #plt.pause(0.1)
        #count =0
        """
        ----------------------------
        do-mpc: Optimizer
        ----------------------------
        """
        # Make one optimizer step (solve the NLP)
        configuration_1.make_step_optimizer()
        print "u_mpc: ", configuration_1.optimizer.u_mpc

        #x=move(configuration_1.simulator.x0_sim,configuration_1.optimizer.u_mpc,1)
        #a1=ax.scatter(x[0],x[1])

        
        
        
        #a2=ax.arrow(x[0],x[1],3*math.cos(x[2]),3*math.sin(x[2]), head_width=0.08, head_length=0.00002)
        #plt.pause(0.1)
        """
        ----------------------------
        do-mpc: Simulator
        ----------------------------
        """
        # Simulate the system one step using the solution obtained in the optimization
        configuration_1.make_step_simulator()

        """
        ----------------------------
        do-mpc: Observer
        ----------------------------
        """
        # Make one observer step
        configuration_1.make_step_observer()

        """
        ------------------------------------------------------
        do-mpc: Prepare next iteration and store information
        ------------------------------------------------------
        """
        # Store the information
        configuration_1.store_mpc_data()

        # Set initial condition constraint for the next iteration
        configuration_1.prepare_next_iter()
        """
        ------------------------------------------------------
        do-mpc: Plot MPC animation if chosen by the user
        ------------------------------------------------------
        """
        # Plot animation if chosen in by the user
        #data_do_mpc.plot_animation(configuration_1)
        #print "dist:", dist(configuration_1.simulator.xf_sim, Y_ref)
        #if dist(configuration_1.simulator.xf_sim, Y_ref)<=5:
        #    break

        """
        ------------------------------------------------------
        do-mpc: Plot the closed-loop results
        ------------------------------------------------------
        """
        #plt.show()
        #data_do_mpc.plot_mpc(configuration_1)

        # Export to matlab if wanted
        #data_do_mpc.export_to_matlab(configuration_1)
        return configuration_1.optimizer.u_mpc
        

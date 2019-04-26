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
    def __init__(self):
        # Create an optimizer object based on the template and a model
        self.optimizer_1 = template_optimizer.optimizer(model_1)
        # Create an observer object based on the template and a model
        self.observer_1 = template_observer.observer(model_1)
        # Create a simulator object based on the template and a model
        self.simulator_1 = template_simulator.simulator(model_1)

    def getOptControl(self)
        # Create the objects for each module
        self.model_1 = template_model.model()
        # Create a configuration
        configuration_1 = core_do_mpc.configuration(model_1, optimizer_1, observer_1, simulator_1)

        # Set up the solvers
        configuration_1.setup_solver()


        """
        ----------------------------
        do-mpc: MPC loop
        ----------------------------
        """
        fig=plt.figure()
        ax=plt.axes()
        plt.ion()
        plt.grid()
        ax.set_xlim((-10, 100))
        ax.set_ylim((-10, 100))

        xo=[60,40]
        rx=15
        ry=10
        elli=patches.Ellipse(xy=xo,width=2*rx, height=2*ry, angle=0)
        ax.add_artist(elli)

        x=[0,0,0]
        Y_ref=[80, 80, 1.57]
        a1=ax.scatter(x[0],x[1])
        a2=ax.arrow(x[0],x[1],3*math.cos(x[2]),3*math.sin(x[2]), head_width=0.08, head_length=0.00002)
        plt.pause(0.1)
        count =0
        while 1:

            """
            ----------------------------
            do-mpc: Optimizer
            ----------------------------
            """
            # Make one optimizer step (solve the NLP)
            configuration_1.make_step_optimizer()
            print "u_mpc: ", configuration_1.optimizer.u_mpc
            
            #a1.remove()
            #a2.remove()
        #    
        #    A=Ad(u, x[n][2])
        #    A_bar=
        #    a=ax.scatter(x[n+1,0],x[n+1,1])
        #    plt.pause(0.5)

            x=move(configuration_1.simulator.x0_sim,configuration_1.optimizer.u_mpc,1)
            a1=ax.scatter(x[0],x[1])

            lam=1e7
            gamma=1
            lambda_Y=np.identity(3)
            
            V=(lam)*exp(-(gamma+((xo[0]-x[0])**2)/(rx**2)+((xo[1]-x[1])**2)/(ry**2)))
            print "cost", V
            print "x cost:",10*(x[0]-Y_ref[0])**2,"y cost:",10*(x[1]-Y_ref[1])**2,"theta cost:", (x[2]-Y_ref[2])**2
            a2=ax.arrow(x[0],x[1],3*math.cos(x[2]),3*math.sin(x[2]), head_width=0.08, head_length=0.00002)
            plt.pause(0.1)
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
            count=count+1
            print "dist:", dist(configuration_1.simulator.xf_sim, Y_ref)
            if dist(configuration_1.simulator.xf_sim, Y_ref)<=5:
                break

        """
        ------------------------------------------------------
        do-mpc: Plot the closed-loop results
        ------------------------------------------------------
        """
        plt.show()
        #data_do_mpc.plot_mpc(configuration_1)

        # Export to matlab if wanted
        #data_do_mpc.export_to_matlab(configuration_1)
        print "count: ", count

        input("Press Enter to exit do-mpc...")

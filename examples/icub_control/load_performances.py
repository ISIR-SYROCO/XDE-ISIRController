#!/usr/bin/env python

import pylab as pl
import json
import numpy as np


def plot_performances( perf ):
    """
    """
    if not isinstance(perf, dict):
        with open(perf, "r") as f:
            perf = json.load(f)
    timeline            = np.array(perf['timeline'])
    orocos_loop_time    = np.array(perf["orocos_loop_time"])
    ctrl_up_tasks       = np.array(perf["controller_update_tasks"])
    ctrl_solve          = np.array(perf["controller_solve_problem"])
    solver_prepare      = np.array(perf["solver_prepare"])
    solver_solve        = np.array(perf["solver_solve"])
    
    step_time = []
    for i in range(len(timeline)-1):
        step_time.append(timeline[i+1] - timeline[i])
    
    
    ##########################
    pl.plot(step_time, label="step time")
    pl.plot(orocos_loop_time, label="orocos loop")
    pl.plot(ctrl_up_tasks, label="ctrl up tasks")
    pl.plot(ctrl_solve, label="ctrl solve problem")
    pl.plot(ctrl_up_tasks+ctrl_solve, label="ctrl task+solve", color="k")
    
    pl.legend()
    pl.ylim(0, pl.ylim()[1])
    pl.title("orocos loop performances")
    pl.ylabel("time (s)")
    pl.xlabel("step")
    
    pl.savefig("./orocos_loop_performances.pdf", bbox_inches='tight')
    
    
    ##########################
    pl.figure()
    pl.plot(ctrl_solve, label="ctrl solve problem", lw=2)
    pl.plot(solver_prepare, label="solver prepare")
    pl.plot(solver_solve, label="solver solve")
    pl.plot(solver_prepare+solver_solve, label="solver prep+solve", color="k")
    
    pl.legend()
    pl.ylim(0, pl.ylim()[1])
    pl.title("controller solver performances")
    pl.ylabel("time (s)")
    pl.xlabel("step")
    
    pl.savefig("./controller_solver_performances.pdf", bbox_inches='tight')
    
    ##########################
    pl.show()

if __name__ == "__main__":
    plot_performances("performances.json")

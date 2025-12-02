from src.sim_environment.devices.ideal import *
from src.sim_environment.optimization_problem import *
from copy import deepcopy
import networkx as nx
import matplotlib.pyplot as plt

def get_test_scenarios():
    #-----------------------------------
    # Scenario 0
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [30, 20, 10, 0, -10, -20, -30, -40, -50, 0]
    c_dev = 5
    max_rel_rand = 0.0
    l = IdealDevice(IdealLoadState(-30), c_load)
    b = IdealDevice(IdealBatteryState(100, 0.5, -20, 20), c_other)
    f = IdealDevice(IdealFuelCellState(100, 20, 10), c_other)
    devices = [l, b, f]
    p0 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))
    # watts strogatz with k = 2 and p = 0 is a ring topology
    t0 = nx.watts_strogatz_graph(len(devices), 2, 0)

    #-----------------------------------
    # Scenario 1
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [0, 0, 80, 80, 20, 50]
    c_dev = 10
    max_rel_rand = 0.0
    l = IdealDevice(IdealLoadState(-30), c_load)
    b = IdealDevice(IdealBatteryState(100, 0.0, -20, 20), c_other)
    f = IdealDevice(IdealFuelCellState(100, 20, 10), c_other)
    devices = [l, deepcopy(l),  b, deepcopy(b), f, deepcopy(f)]
    p1 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))
    t1 = nx.watts_strogatz_graph(len(devices), 2, 0)
    #nx.draw(t1, with_labels=True, font_weight='bold')
    #plt.draw()
    #plt.show()
    #-----------------------------------
    # Scenario 2
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [0, 0, 80, 80, 20, 50]
    c_dev = 10
    max_rel_rand = 0.0
    l = IdealDevice(IdealLoadState(-30), c_load)
    b = IdealDevice(IdealBatteryState(100, 0.0, -20, 20), c_other)
    f = IdealDevice(IdealFuelCellState(100, 20, 10), c_other)
    devices = [l, deepcopy(l), deepcopy(l),deepcopy(l), b, deepcopy(b), deepcopy(b),deepcopy(b), f, deepcopy(f),deepcopy(f), deepcopy(f)]
    p2 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))
    t2 = nx.watts_strogatz_graph(len(devices), 2, 1)
    #nx.draw(t2, with_labels=True, font_weight='bold')
    #plt.draw()
    #plt.show()
    return [(p0, t0), (p1, t1),  (p2, t2)]
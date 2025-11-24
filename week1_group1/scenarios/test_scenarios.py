from week2_group1.src.sim_environment.devices.ideal import *
from week2_group1.src.sim_environment.optimization_problem import *
from copy import deepcopy

def get_test_scenarios():
    #-----------------------------------
    # Scenario 0: nothing complicated, no randomization
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

    #-----------------------------------
    # Scenario 1: double device size and some randomization
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [0.9 * x for x in [60, 40, 20, 0, -20, -40, -60, -80, -100, 0, 0, 0, 0]]
    c_dev = 10
    max_rel_rand = 0.1
    l = IdealDevice(IdealLoadState(-30), c_load)
    b = IdealDevice(IdealBatteryState(200, 0.5, -30, 30), c_other)
    f = IdealDevice(IdealFuelCellState(100, 30, 5), c_other)
    devices = [l, deepcopy(l),  b, deepcopy(b), f, deepcopy(f)]
    p1 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))

    #-----------------------------------
    # Scenario 2: same devices as scenario 1 but longer schedule
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [0.9 * x for x in 
             [60, 40, 20, 0, -20, -40, -60, -80, -100, 0, 
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0, 
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0,
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0,
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0,
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0,
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0,
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0,
              60, 40, 20, 0, -20, -40, -60, -80, -100, 0,]]
    c_dev = 10
    max_rel_rand = 0
    l = IdealDevice(IdealLoadState(-30), c_load)
    b = IdealDevice(IdealBatteryState(150, 0.5, -30, 30), c_other)
    f = IdealDevice(IdealFuelCellState(150, 30), c_other)
    devices = [l, deepcopy(l),  b, deepcopy(b), f, deepcopy(f)]
    p2 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))

    #-----------------------------------
    # Scenario 3: constructed to require explicit pre-loading of the battery
    # to meet the power demand at later stages
    # simple calculation:
    # - fuel cells can provide 40 peak power
    # - batteries can supply another 40 but start empty
    # - schedule starts with no power requirement, time to charge battery from fuel
    # - schedule stops with positive power to unload the battery to meet the t(0) = t(end) constraint
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
    p3 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))


    #-----------------------------------
    # Scenario 4: less tight scheduling that should be
    # solvable with 0 deviation cost.
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [0, 10, 20, 30, 40, 50, -50, -40, -30, -20, -10, 0]
    c_dev = 10
    max_rel_rand = 0.05
    l = IdealDevice(IdealLoadState(-60), c_load)
    b = IdealDevice(IdealBatteryState(400, 0.0, -40, 40), c_other)
    f = IdealDevice(IdealFuelCellState(400, 40), c_other)
    devices = [l, deepcopy(l),  b, deepcopy(b), f, deepcopy(f)]
    p4 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))

    #-----------------------------------
    # Scenario 5: very heavy randomization, plenty of devices
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [0, 10, 20, 30, 40, 50, -50, -40, -30, -20, -10, 0]
    c_dev = 5
    max_rel_rand = 0.8
    l = IdealDevice(IdealLoadState(-60), c_load)
    b = IdealDevice(IdealBatteryState(400, 0.0, -40, 40), c_other)
    f = IdealDevice(IdealFuelCellState(400, 40), c_other)
    devices = [l, deepcopy(l), deepcopy(l), b, deepcopy(b), deepcopy(b), f, deepcopy(f), deepcopy(f)]
    p5 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))


    #-----------------------------------
    # Scenario 6: loads only
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [-10, -20, -30, -40, -50, -60, -70, -80, -90, 
              -10, -20, -30, -40, -50, -60, -70, -80, -90]
    c_dev = 5
    max_rel_rand = 0.4
    l = IdealDevice(IdealLoadState(-60), c_load)
    devices = [l, deepcopy(l), deepcopy(l)]
    p6 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))

    #-----------------------------------
    # Scenario 7: fuel cells only
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [100, 80, 10, 100, 80, 10, 100, 80, 10]
    c_dev = 5
    max_rel_rand = 0.2
    f = IdealDevice(IdealFuelCellState(400, 40), c_other)
    devices = [f, deepcopy(f), deepcopy(f)]
    p7 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))

    #-----------------------------------
    # Scenario 8: batteries only
    #-----------------------------------
    c_load = 0.5
    c_other = 0.7
    target = [100, 80, 10, -100, -80, -10, 0.0, 0.0, 0.0, 40, -40]
    c_dev = 5
    max_rel_rand = 0.0
    b = IdealDevice(IdealBatteryState(400, 0.5, -40, 40), c_other)
    devices = [b, deepcopy(b), deepcopy(b)]
    p8 = deepcopy(SchedulingProblem(target, devices, c_dev, max_rel_rand))

    return [p0, p1, p2, p3, p4, p5, p6, p7, p8]
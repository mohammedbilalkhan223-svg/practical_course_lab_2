"""
Hints:
- Implement the ED_solve function first, it will be useful to decide unit commitment.
- Scenarios contain different sets of devices. You must parse these and set constraints for each.
- The file gives a suggestion on how to:
    - organize your code
    - handle generic names for attributed in pyomo

The device state objects (contained in device.state) have the following  relevant fields:

IdealBatteryState
- size
- soc
- final_soc (the SOC it should have at the end!)
- p_max
- p_min

IdealLoadState
- p_min
- p_max

IdealFuelCellState
- fuel_amount
- p_min
- p_max
- change_max
- p_prev 
    (the power value in the time step before the scheduling start)
"""
from src.sim_environment.devices.ideal import *

from src.sim_environment.devices.hil import IdealBatteryState as HILBatteryState
from src.sim_environment.devices.hil import IdealLoadState as HILloadstate
from src.sim_environment.devices.hil import IdealFuelCellState as HILFuelState

import logging
import pyomo.environ as pyo
from copy import deepcopy
import itertools
from collections import defaultdict


# decide if a device.state object is for a specific kind of device
def is_battery_state(obj):
    return isinstance(obj, IdealBatteryState) or isinstance(obj, HILBatteryState)


def is_load_state(obj):
    return isinstance(obj, IdealLoadState) or isinstance(obj, HILloadstate)


def is_fuel_cell_state(obj):
    return isinstance(obj, IdealFuelCellState) or isinstance(obj, HILFuelState)


# Generic naming functions to set pyomo model variables
def power_name(device_number):
    return f"power_{device_number}"


def cost_name(device_number):
    return f"cost_{device_number}"


def const_list_name(device_number):
    return f"constraints_{device_number}"


# creating model and adding different parts
def get_pyomo_model(devices, committed_units, target, c_dev):
    # initialize a new pyomo model
    model = pyo.ConcreteModel()

    # store meta info on the model
    model._devices = devices
    model._committed_units = committed_units
    model._target = list(target) #list of target values
    model._c_dev = float(c_dev) # deviation cost
    model._n_steps = len(target) #-> number of timesteps
    model._n_devices = len(devices) #->device amount

    # sets
    model.T = pyo.RangeSet(0, model._n_steps - 1)  # power timesteps
    model.T_state = pyo.RangeSet(0, model._n_steps)  # state timesteps (for E, to enable cyclic behavior)
    model.devices_idx = pyo.RangeSet(0, model._n_devices - 1)  # device indices 0 to n-1 (total of n devices)

    # classify devices by type
    load_idx = [i for i, d in enumerate(devices) if is_load_state(d.state)]
    batt_idx = [i for i, d in enumerate(devices) if is_battery_state(d.state)]
    fuel_idx = [i for i, d in enumerate(devices) if is_fuel_cell_state(d.state)]
    model.loads = pyo.Set(initialize=load_idx)
    model.batteries = pyo.Set(initialize=batt_idx)
    model.fuelcells = pyo.Set(initialize=fuel_idx)

    # add constraints that exist independent of the chosen devices
    model = add_problem_constraints(model, batt_idx, fuel_idx)
    # add device specific constraints
    model = add_device_constraints(model)
    # add objective function
    model = add_objective_function(model)

    return model


def add_problem_constraints(model, batt_idx, fuel_idx):

    # variables
    # P[i,t] Power to be optimizes (positive and/or negative depending on device)
    model.P = pyo.Var(model.devices_idx, model.T, domain=pyo.Reals)

    # absP[i,t] absolute value of P to calculate operating costs in cost function
    model.Pabs = pyo.Var(model.devices_idx, model.T, domain=pyo.NonNegativeReals)

    # PDelta[t] deviation between target and optimized power
    model.PDelta = pyo.Var(model.T, domain=pyo.NonNegativeReals)

    # battery energies E[b,t]
    if batt_idx:
        model.E = pyo.Var(model.batteries, model.T_state,
                          domain=pyo.NonNegativeReals)

    # fuel amounts F[f,t]
    if fuel_idx:
        model.F = pyo.Var(model.fuelcells, model.T_state,
                          domain=pyo.NonNegativeReals)

    # absolute value of operating cost (due to lineraization)
    def Pabs_upper_rule(m, i, t):
        return m.Pabs[i, t] >= m.P[i, t]

    def Pabs_lower_rule(m, i, t):
        return m.Pabs[i, t] >= -m.P[i, t]
    model.Pabs_upper = pyo.Constraint(model.devices_idx, model.T, rule=Pabs_upper_rule)
    model.Pabs_lower = pyo.Constraint(model.devices_idx, model.T, rule=Pabs_lower_rule)

    # absolute value of deviation between target and total power at timestep (due to lineraization)
    def Pdelta_upper_rule(m, t):
        return m.PDelta[t] >= m._target[t] - sum(m.P[i, t] for i in m.devices_idx)

    def Pdelta_lower_rule(m, t):
        return m.PDelta[t] >= -(m._target[t] - sum(m.P[i, t] for i in m.devices_idx))
    model.Pdelta_upper = pyo.Constraint(model.T, rule=Pdelta_upper_rule)
    model.Pdelta_lower = pyo.Constraint(model.T, rule=Pdelta_lower_rule)
    return model


def add_device_constraints(model):
    # constraint lists per device
    model.device_constraints = pyo.ConstraintList()

    for i, dev in enumerate(model._devices):
        state = dev.state
        committed = model._committed_units[i]

        if not committed:
            for t in model.T:
                model.device_constraints.add(model.P[i, t] == 0.0) #setting not commited device constraint so that P = 0 for not commited devices
            continue

        # add constraints depending on device type
        if is_load_state(state):
            add_load_state_constraints(model, i, state)
        elif is_battery_state(state):
            add_battery_state_constraints(model, i, state)
        elif is_fuel_cell_state(state):
            add_fuel_cell_state_constraints(model, i, state)
        else:
            logging.warning(f"Unknown device type at index {i}, no constraints added.")

    return model


def add_objective_function(model):
    """
    Objective: minimize sum_t c_dev * D[t]  +  sum_{i,t} c_op_i * U[i,t]
    """
    def obj_rule(m):
        total = 0.0
        for t in m.T:
            total += m._c_dev * m.PDelta[t] # deviation cost for deviation between target and optimized value
        for i, dev in enumerate(m._devices):
            for t in m.T:
                total += dev.c_op * m.Pabs[i, t] # operating cost (device specific), going through devices and summing over time
        for i, dev in enumerate(m._devices):
            committed = m._committed_units[i]
            if committed:
                total += dev.commitment_cost #commitment cost for commited units
        return total
    model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)
    return model


def add_load_state_constraints(model, device_index, load_state):
    """
    IdealLoadState:
    - p_min <= P(i,t) <= p_max (with p_max usually = 0)
    """
    for t in model.T:
        model.device_constraints.add(pyo.inequality(float(load_state.p_min), model.P[device_index, t], float(load_state.p_max)))

def add_battery_state_constraints(model, device_index, battery_state):
    """
    IdealBatteryState:
    - P_min <= P(i,t) <= P_max
    - E(i,0) = size * soc
    - E(i,T) = size * final_soc
    - 0 <= E(i,t) <= size
    - E(i,t+1) = E(i,t) - P(i,t)
    - E(i,T+1) = E(t=0)
    """

    for t in model.T:
        model.device_constraints.add(pyo.inequality(float(battery_state.p_min), model.P[device_index, t], float(battery_state.p_max))) #Power bounds

    model.device_constraints.add(model.E[device_index, 0] == float(battery_state.size) * float(battery_state.soc)) # initial energy
    model.device_constraints.add(model.E[device_index, model._n_steps] == float(battery_state.size) * float(battery_state.final_soc))    # final energy (SOC constraint), last value of E is initial value (cyclic behaviour of battery)
    for t in model.T_state:
        model.device_constraints.add(model.E[device_index, t] <= float(battery_state.size)) # energy bounds
        # lower bound 0 is enforced by NonNegativeReals domain
    for t in range(model._n_steps):
        model.device_constraints.add(model.E[device_index, t+1] == model.E[device_index, t] - model.P[device_index, t]) # E in next timestep is E current ts - used power in ts

def add_fuel_cell_state_constraints(model, device_index, fuel_cell_state):
    """
    IdealfuelcellState:
    - 0 <= P(i,t) <= p_max
    - F(i,0) = fuel_amount
    - F(i,t+1) = F(i,t) - P(i,t)
    - F(i,t) >= 0 (via NonNegativeReals)
    - ramp: |P(i,t+1) - P(i,t)| <= change_max
            |P(i,0) - p_prev| <= change_max
    """

    for t in model.T:
        model.device_constraints.add(pyo.inequality(float(fuel_cell_state.p_min), model.P[device_index, t], float(fuel_cell_state.p_max))) #adding power bounds
    model.device_constraints.add(model.F[device_index, 0] == float(fuel_cell_state.fuel_amount)) # initial fuel
    # dynamics and power bounds
    for t in range(model._n_steps):
        # fuel evolution
        model.device_constraints.add(model.F[device_index, t + 1] == model.F[device_index, t] - model.P[device_index, t]) #Fuel amount in next ts is current fuel amount minus Power used in ts
        # fuel >= 0 is enforced by NonNegativeReals

    model.device_constraints.add(model.P[device_index, 0] - float(fuel_cell_state.p_prev) <= float(fuel_cell_state.change_max)) # ramp from previous power to first step in absolute value
    model.device_constraints.add(-(model.P[device_index, 0] - float(fuel_cell_state.p_prev)) <= float(fuel_cell_state.change_max)) #'''
    if model._n_steps > 1: # ramp between consecutive time steps
        for t in range(0, model._n_steps - 1):
            model.device_constraints.add(model.P[device_index, t + 1] - model.P[device_index, t] <= float(fuel_cell_state.change_max))
            model.device_constraints.add( model.P[device_index, t] - model.P[device_index, t + 1] <= float(fuel_cell_state.change_max))


def ED_solve(devices, committed_units, target, c_dev):
    # ensure no operation here accidentally alters the device objects
    updated_devices = deepcopy(devices)

    # build pyomo model
    model = get_pyomo_model(updated_devices, committed_units, target, c_dev)

    # solve with HiGHS (appsi interface)
    solver = pyo.SolverFactory("appsi_highs")
    result = solver.solve(model, load_solutions=True)

    # optional: check solver status
    if (not hasattr(result, "solver")) or (str(result.solver.termination_condition) != "optimal"):
        logging.info(f"Solver termination condition: {getattr(result.solver, 'termination_condition', 'Unknown')}")

    n_dev = len(updated_devices)
    n_steps = len(target)

    # extract schedules [device][t]
    power_schedules = []
    for i in range(n_dev):
        sched = [pyo.value(model.P[i, t]) for t in model.T]
        # safety: ensure length matches
        if len(sched) != n_steps:
            sched = [pyo.value(model.P[i, t]) for t in range(n_steps)]
        power_schedules.append(sched)

    # schedule_cost: objective value (deviation + operating cost)
    schedule_cost = pyo.value(model.obj)

    return power_schedules, schedule_cost
"""
Input:
- devices: List of IdealDevice objects
- target: the list of target values
- c_dev: the target difference cost factor

Output:
- List of {True, False} of same length as devices indicating which are to be committed
  e.g. for 6 devices it could be:
  [True, True, False, True, False False]
  meaning devices at indices 0,1, and 3 are to be used.
"""
def UC_solve(devices, target, c_dev):
    """
    Unit Commitment (UC) solver: decide which devices to commit (use) for the day-ahead schedule.

    Strategy:
    - For each device, evaluate whether committing it leads to a lower total cost (selection cost + operating cost + deviation cost).
    - Use a greedy heuristic: start with no devices committed, then iteratively add the device that provides the largest cost reduction.
    - Use ED_solve to compute the cost for each candidate set of committed devices.
    - Stop when adding more devices no longer reduces the total cost.

    This is a heuristic approach to avoid the exponential search over 2^n combinations.

    Args:
        devices: List of device objects (with .c_op, .c_sel, and .state attributes)
        target: List of target power values for each time step
        c_dev: Deviation cost factor (c_dev)

    Returns:
        List of booleans indicating whether each device is committed (True) or not (False)
    """
    n_dev = len(devices)
    counter = 0 #counter for total iterations
    best_commitment = None
    best_cost = None
    i = None #counter for unchanging iterations
    device_types = []
    for d in devices:
        p_span = d.state.p_max - d.state.p_min
        if p_span == 0:
            d.rank_val = float('inf')
        else:
            d.rank_val = d.commitment_cost / p_span + d.c_op

    for dev in devices:
        if is_load_state(dev.state):
            type = "load"
        elif is_battery_state(dev.state):
            type = "battery"
        elif is_fuel_cell_state(dev.state):
            type = "fuel_cell"
        else:
            type = "unknown"
        device_types.append((type, dev.rank_val))  # getting list with type and rank value

    all_combinations = [list(combi) for combi in itertools.product([False,True], repeat = n_dev)]
    all_combinations = [row for row in all_combinations if any(row)]
    costs = [cost for _, cost in device_types]
    type_names = [t for t, _ in device_types]
    unique_type_names = list(dict.fromkeys(type_names))
    unique_rows = {}
    target_pmax = max(target)
    target_pmin = min(target)

    for set in all_combinations:
        total_cost_pu = sum(cost for flag, cost in zip(set, costs) if flag)
        total_pmax = sum(dev.state.p_max for flag, dev in zip(set, devices) if flag)
        total_pmin = sum(dev.state.p_min for flag, dev in zip(set, devices) if flag)

        counts = defaultdict(int)
        for flag,typ in zip(set, type_names):
            if flag:
                counts[typ]+=1
        amount_vec = tuple(counts[t] for t in unique_type_names) #creates "list" with counter of unique type, here unique load, battery,fuel cell -> 3 entries to then

        if amount_vec not in unique_rows: #if amount_vec not in unique rows yet -> to only include one of the same combinaiton of devices
            unique_rows[amount_vec] = {
                "set": set,  # list of booleans for committed or not
                "cost_pu": total_cost_pu,  # selection‑cost per unit + operation cost
                "pmax": total_pmax,
                "pmin": total_pmin,
                "types": type_names,
                "diff_max": None,
                "diff_min": None,
                "total_cost_max": None,
                "total_cost_min": None,
                "total_cost": None,
                "feasible": None
            }
    unique_combinations_sorted = sorted(unique_rows.values(), key = lambda x: x["cost_pu"])

    feasible_combinations = []
    for combination in unique_combinations_sorted:
        pmax = combination["pmax"]
        pmin = combination["pmin"]
        cost_pu = combination["cost_pu"]
        diff_max = target_pmax - pmax
        diff_min = target_pmin - pmin #target_min might be negative, p_min might be negative (in our cases always <=0)
        if diff_max < 0: #more p providable in combination
            total_cost_max = target_pmax * cost_pu
            feasible = True
        elif diff_max >= 0: #not enough p in combination
            total_cost_max = pmax * cost_pu + diff_max * c_dev
            feasible = False
        if diff_min < 0: #less load in combination than needed
            total_cost_min = pmin * cost_pu + diff_min * c_dev
            feasible = False
        elif diff_min >= 0: #more load in combination than needed
            total_cost_min = target_pmin * cost_pu
            feasible = True
        total_cost = abs(total_cost_max) + abs(total_cost_min) #sum up absolute values for total cost
        combination.update(
            {
                "diff_max": diff_max,
                "diff_min": diff_min,
                "total_cost_max": total_cost_max,
                "total_cost_min": total_cost_min,
                "total_cost": total_cost,
                "feasible": feasible
            })
        feasible_combinations.append(combination)
    # now feasibility check -> feasible and not feasible but cheapest combinations are selected
    feasible = [c for c in feasible_combinations if c["feasible"]]
    infeasible = [c for c in feasible_combinations if not c["feasible"]]
    if feasible:
        min_feasible_cost = min(c["total_cost"] for c in feasible)
    else:
        min_feasible_cost = float('inf')
        print("No feasible combination – all infeasible combos will be kept")
    cheap_infeasible = [c for c in infeasible if c["total_cost"] < min_feasible_cost]
    feasible_combinations = feasible + cheap_infeasible
    num_feasible = sum(1 for entry in feasible_combinations if entry["feasible"])
    num_infeasible = len(feasible_combinations) - num_feasible
    feasible_combinations = sorted(feasible_combinations, key = lambda x: x["total_cost"]) #sort for total cost to check cheapest ones first

    # going through filtered options
    print("---------------- checking feasible combinations with total length: ", len(feasible_combinations))
    for option in feasible_combinations:
        committed_units = option["set"]
        counter += 1
        power_schedules, schedule_cost = ED_solve(devices, committed_units, target, c_dev)
        if best_cost is None or schedule_cost <= best_cost:
            best_cost = schedule_cost
            best_commitment = committed_units
            counter_best = counter
            i = 0
        elif i > len(feasible_combinations)/2:
            print("No improvement within the last", len(feasible_combinations)/2,"  iterations")
            break
        else:
            i += 1
    print("best_cost with feasible solutions", best_cost, "with:", best_commitment, "counter was at:", counter_best, "no changes for = ", i, "iterations")
    print("counter", counter)
    return best_commitment
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
    model._committed = committed_units
    model._target = target
    model._c_dev = c_dev
    model._n_steps = len(target)
    model._n_dev = len(devices)

    # add constraints that exist independent of the chosen devices
    model = add_problem_constraints(model, devices, committed_units, target, c_dev)

    # add device specific constraints
    model = add_device_constraints(model, devices, committed_units)

    # add objective function
    model = add_objective_function(model)

    return model


def add_problem_constraints(model, devices, committed_units, target, c_dev):
    n_steps = len(target)  # number of timesteps
    n_dev = len(devices)  # number of devices

    # sets
    model.T = pyo.RangeSet(0, n_steps - 1)  # power timesteps
    model.T_state = pyo.RangeSet(0, n_steps)  # state timesteps (E/F)
    model.devices = pyo.RangeSet(0, n_dev - 1)  # device indices

    # classify devices by type
    load_idx = [i for i, d in enumerate(devices) if is_load_state(d.state)]
    batt_idx = [i for i, d in enumerate(devices) if is_battery_state(d.state)]
    fuel_idx = [i for i, d in enumerate(devices) if is_fuel_cell_state(d.state)]

    model.loads = pyo.Set(initialize=load_idx)
    model.batteries = pyo.Set(initialize=batt_idx)
    model.fuelcells = pyo.Set(initialize=fuel_idx)

    # variables
    # P[i,t] Power to be optimizes (positive and/or negative depending on device)
    model.P = pyo.Var(model.devices, model.T, domain=pyo.Reals)

    # U[i,t] absolute value of P to calculate operating costs in cost function
    model.U = pyo.Var(model.devices, model.T, domain=pyo.NonNegativeReals)

    # D[t] deviation between target and optimized value
    model.D = pyo.Var(model.T, domain=pyo.NonNegativeReals)

    # battery energies E[b,t]
    if batt_idx:
        model.E = pyo.Var(model.batteries, model.T_state,
                          domain=pyo.NonNegativeReals)

    # fuel amounts F[f,t]
    if fuel_idx:
        model.F = pyo.Var(model.fuelcells, model.T_state,
                          domain=pyo.NonNegativeReals)

    # absolute value of deviation between target and total power at timestep
    def dev_upper_rule(m, t):
        total_p = sum(m.P[i, t] for i in m.devices)
        return m.D[t] >= m._target[t] - total_p

    def dev_lower_rule(m, t):
        total_p = sum(m.P[i, t] for i in m.devices)
        return m.D[t] >= -(m._target[t] - total_p)

    model.dev_upper = pyo.Constraint(model.T, rule=dev_upper_rule)
    model.dev_lower = pyo.Constraint(model.T, rule=dev_lower_rule)

    # absolute value of operating cost
    def u_upper_rule(m, i, t):
        return m.U[i, t] >= m.P[i, t]

    def u_lower_rule(m, i, t):
        return m.U[i, t] >= -m.P[i, t]

    model.u_upper = pyo.Constraint(model.devices, model.T, rule=u_upper_rule)
    model.u_lower = pyo.Constraint(model.devices, model.T, rule=u_lower_rule)

    return model


def add_device_constraints(model, devices, committed_units):
    # constraint lists per device
    model.device_constraints = pyo.ConstraintList()

    for i, dev in enumerate(devices):
        state = dev.state
        committed = committed_units[i]

        if not committed:
            for t in model.T:
                model.P[i, t].fix(0.0)
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

    def obj_rule(m, co):
        total = 0.0

        # deviation cost for deviation between target and optimized value
        for t in m.T:
            total += m._c_dev * m.D[t]

        # operating cost (device specific), going through devices and summing over time
        for i, dev in enumerate(m._devices):
            for t in m.T:
                total += dev.c_op * m.U[i, t]

        for i, dev in enumerate(m._devices):
            committed = m._committed[i]
            if committed:
                total += dev.commitment_cost
        return total

    model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)
    return model


def add_load_state_constraints(model, dev_index, load_state):
    """
    IdealLoadState:
    - p_min <= P(i,t) <= p_max (with p_max usually = 0)
    """
    const_list = pyo.ConstraintList()
    setattr(model, const_list_name(dev_index), const_list)

    for t in model.T:
        const_list.add(model.P[dev_index, t] >= load_state.p_min)
        const_list.add(model.P[dev_index, t] <= load_state.p_max)


def add_battery_state_constraints(model, dev_index, battery_state):
    """
    IdealBatteryState:
    - P_min <= P(i,t) <= P_max
    - E(i,0) = size * soc
    - E(i,T) = size * final_soc
    - 0 <= E(i,t) <= size
    - E(i,t+1) = E(i,t) - P(i,t)
    - E(i,t+1) = E(t=0)
    """
    const_list = pyo.ConstraintList()
    setattr(model, const_list_name(dev_index), const_list)

    # initial energy
    const_list.add(model.E[dev_index, 0] == battery_state.size * battery_state.soc)

    # final energy (SOC constraint)
    const_list.add(model.E[dev_index, model._n_steps] == battery_state.size * battery_state.final_soc)

    # energy bounds
    for t in model.T_state:
        const_list.add(model.E[dev_index, t] <= battery_state.size)
        # lower bound 0 is enforced by NonNegativeReals domain
    const_list.add(model.E[dev_index, model.T[-1] + 1] == model.E[dev_index, 0])

    # dynamics + power limits
    for t in model.T:
        const_list.add(model.E[dev_index, t + 1] == model.E[dev_index, t] - model.P[dev_index, t])
        const_list.add(model.P[dev_index, t] >= battery_state.p_min)
        const_list.add(model.P[dev_index, t] <= battery_state.p_max)


def add_fuel_cell_state_constraints(model, dev_index, fuel_cell_state):
    """
    IdealfuelcellState:
    - 0 <= P(i,t) <= p_max
    - F(i,0) = fuel_amount
    - F(i,t+1) = F(i,t) - P(i,t)
    - F(i,t) >= 0 (via NonNegativeReals)
    - ramp: |P(i,t+1) - P(i,t)| <= change_max
            |P(i,0) - p_prev| <= change_max
    """
    const_list = pyo.ConstraintList()
    setattr(model, const_list_name(dev_index), const_list)

    # initial fuel
    const_list.add(model.F[dev_index, 0] == fuel_cell_state.fuel_amount)

    # dynamics and power bounds
    for t in model.T:
        # fuel evolution
        const_list.add(
            model.F[dev_index, t + 1] == model.F[dev_index, t] - model.P[dev_index, t]
        )
        # power limits
        const_list.add(model.P[dev_index, t] >= fuel_cell_state.p_min)
        const_list.add(model.P[dev_index, t] <= fuel_cell_state.p_max)
        # fuel >= 0 is enforced by NonNegativeReals

    # ramp from previous power to first step
    p_prev = float(fuel_cell_state.p_prev)
    const_list.add(model.P[dev_index, 0] - p_prev <= fuel_cell_state.change_max)
    const_list.add(p_prev - model.P[dev_index, 0] <= fuel_cell_state.change_max)

    # ramp between consecutive time steps
    if model._n_steps > 1:
        for t in range(0, model._n_steps - 1):
            const_list.add(
                model.P[dev_index, t + 1] - model.P[dev_index, t] <= fuel_cell_state.change_max
            )
            const_list.add(
                model.P[dev_index, t] - model.P[dev_index, t + 1] <= fuel_cell_state.change_max
            )


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
            return float('inf')
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
        elif i > 8:
            print("No improvement within the last 9 iterations")
            break
        else:
            i += 1
    print("best_cost with feasible solutions", best_cost, "with:", best_commitment, "counter was at:", counter_best, "no changes for = ", i, "iterations")
    print("counter", counter)
    return best_commitment

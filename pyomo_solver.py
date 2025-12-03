from src.sim_environment.devices.ideal import *
# Imports everything (*) defined in src.sim_environment.devices.ideal.
# That includes classes like IdealDevice, IdealBatteryState, IdealLoadState, IdealFuelCellState, and any other symbols defined in that module.
# This allows the file to reference the ideal device classes directly.
from src.sim_environment.devices.hil import IdealBatteryState as HILBatteryState
from src.sim_environment.devices.hil import IdealLoadState as HILloadstate
from src.sim_environment.devices.hil import IdealFuelCellState as HILFuelState
# Imports hardware-in-the-loop (HIL) variants of the device state classes from devices.hil. Each import renames the class locally:
# IdealBatteryState → HILBatteryState
# IdealLoadState → HILloadstate
# IdealFuelCellState → HILFuelState
# These aliases let the module treat HIL and ideal states interchangeably by checking types against either set of classes.
import logging
# Imports Python’s logging module to report warnings/info messages instead of printing directly.
import pyomo.environ as pyo
# Imports the Pyomo modeling environment under the alias pyo. Pyomo is used to create optimization models,
# define variables/constraints, and solve them with solvers.
from copy import deepcopy
# Imports deepcopy so device objects can be duplicated safely (changes to copies won’t affect originals).
import itertools
# Imports the itertools module, used later for generating all combinations of committed/uncommitted devices.
from collections import defaultdict
# Imports defaultdict (a dictionary variant that provides a default value for missing keys), used to count device types in combinations.

# decide if a device.state object is for a specific kind of device


def is_battery_state(obj):
    return isinstance(obj, IdealBatteryState) or isinstance(obj, HILBatteryState)
# Defines helper is_battery_state(obj) that returns True if obj is either the ideal battery state class or the HIL battery state class.
# This allows the solver to detect battery devices regardless of implementation source.


def is_load_state(obj):
    return isinstance(obj, IdealLoadState) or isinstance(obj, HILloadstate)
# Similar helper for load states. Returns True if obj is IdealLoadState or the HIL load state type.


def is_fuel_cell_state(obj):
    return isinstance(obj, IdealFuelCellState) or isinstance(obj, HILFuelState)
# Similar helper for fuel cell states.

# Generic naming functions to set pyomo model variables


def power_name(device_number):
    return f"power_{device_number}"
# A small utility that formats a variable name for a device’s power (not used elsewhere in the code but provided for consistency).
# Returns a string like "power_0".


def cost_name(device_number):
    return f"cost_{device_number}"
# Utility to format cost variable name for a device, e.g., "cost_1". Again, present for naming consistency.


def const_list_name(device_number):
    return f"constraints_{device_number}"
# Utility to format constraints list name for a device, e.g., "constraints_2".

# creating model and adding different parts


def get_pyomo_model(devices, committed_units, target, c_dev):
    # Defines the main function that constructs a Pyomo optimization model from input:
    # initialize a new pyomo model
    model = pyo.ConcreteModel()

    # store meta info on the model
    model._devices = devices
    model._committed_units = committed_units
    model._target = list(target)  # list of target values
    model._c_dev = float(c_dev)  # deviation cost
    model._n_steps = len(target)  # -> number of timesteps
    model._n_devices = len(devices)  # ->device amount

    # sets
    model.T = pyo.RangeSet(0, model._n_steps - 1)  # power timesteps
# T: timesteps where power is defined (0 .. n_steps-1)
    # state timesteps (for E, to enable cyclic behavior)
    model.T_state = pyo.RangeSet(0, model._n_steps)
# T_state: state timesteps, includes an extra index n_steps to store end-of-day state (for energy/fuel variables) and support cyclic constraints (initial = final)
    # device indices 0 to n-1 (total of n devices)
    model.devices_idx = pyo.RangeSet(0, model._n_devices - 1)
# indices for devices (0 .. n_devices-1)
    # classify devices by type
    load_idx = [i for i, d in enumerate(devices) if is_load_state(d.state)]
    batt_idx = [i for i, d in enumerate(devices) if is_battery_state(d.state)]
    fuel_idx = [i for i, d in enumerate(
        devices) if is_fuel_cell_state(d.state)]
    model.loads = pyo.Set(initialize=load_idx)
    model.batteries = pyo.Set(initialize=batt_idx)
    model.fuelcells = pyo.Set(initialize=fuel_idx)
# Classifies device indices into lists based on their state type:
# load_idx: indices of all loads
# batt_idx: indices of all batteries
# fuel_idx: indices of all fuel cells
# Creates Pyomo Set objects model.loads, model.batteries, and model.fuelcells initialized with those index lists.
# Those sets help define variables that exist only for certain device types (e.g., battery energy variables).

    # add constraints that exist independent of the chosen devices
    model = add_problem_constraints(model, batt_idx, fuel_idx)
# Calls add_problem_constraints to add variables and global constraints common to all problems (e.g., power variables, absolute value linearizations, energy/fuel variables).
# Passes battery and fuel index lists so that energy/fuel variables are created only if needed.
    # add device specific constraints
    model = add_device_constraints(model)
# Calls add_device_constraints which iterates over the devices and adds constraints tailored to each device type (bounds, dynamics, ramping).
    # add objective function
    model = add_objective_function(model)
# Adds the objective function (minimize deviation cost + operating costs + commitment costs).
    return model


def add_problem_constraints(model, batt_idx, fuel_idx):
    # Defines a function to add global problem-level variables and constraints to the provided model. batt_idx and fuel_idx indicate if battery/fuel variables are needed.
    # variables
    # P[i,t] Power to be optimizes (positive and/or negative depending on device)
    model.P = pyo.Var(model.devices_idx, model.T, domain=pyo.Reals)
# Declares a Pyomo variable P[i,t] for each device i and timestep t. Domain Reals allows positive (generation) and negative (consumption) values depending on device.
    # absP[i,t] absolute value of P to calculate operating costs in cost function
    model.Pabs = pyo.Var(model.devices_idx, model.T,
                         domain=pyo.NonNegativeReals)
# Adds Pabs[i,t], the nonnegative variable representing |P[i,t]|. This is used to model linear operating costs that depend on absolute energy delivered/consumed.
    # PDelta[t] deviation between target and optimized power
    model.PDelta = pyo.Var(model.T, domain=pyo.NonNegativeReals)
# Adds PDelta[t], the nonnegative variable representing |target[t] - sum_i P[i,t]|. It captures deviation magnitude used in the deviation penalty.
    # battery energies E[b,t]
    if batt_idx:
        model.E = pyo.Var(model.batteries, model.T_state,
                          domain=pyo.NonNegativeReals)
# If there are any batteries (i.e., batt_idx non-empty), declares E[b,t] variables for battery stored energy across state timesteps T_state.
# Domain NonNegativeReals ensures energy ≥ 0.
    # fuel amounts F[f,t]
    if fuel_idx:
        model.F = pyo.Var(model.fuelcells, model.T_state,
                          domain=pyo.NonNegativeReals)
# If any fuel cells exist, declares F[f,t] variables for remaining fuel amount across T_state. Also nonnegative.
    # absolute value of operating cost (due to lineraization)

    def Pabs_upper_rule(m, i, t):
        return m.Pabs[i, t] >= m.P[i, t]
# Defines a rule function Pabs_upper_rule for a constraint ensuring Pabs >= P.
# This is one side of the constraints that enforce Pabs = |P|: since Pabs is nonnegative, two inequalities suffice to force Pabs >= |P|, and the objective will minimize Pabs (via costs) so it becomes equal.

    def Pabs_lower_rule(m, i, t):
        return m.Pabs[i, t] >= -m.P[i, t]
# The complementary rule Pabs >= -P. Together with the previous one, Pabs >= |P|.
    model.Pabs_upper = pyo.Constraint(
        model.devices_idx, model.T, rule=Pabs_upper_rule)
    model.Pabs_lower = pyo.Constraint(
        model.devices_idx, model.T, rule=Pabs_lower_rule)
# Instantiates the two constraint families (Pabs_upper and Pabs_lower) over all device indices and timesteps using the defined rules.
    # absolute value of deviation between target and total power at timestep (due to lineraization)

    def Pdelta_upper_rule(m, t):
        return m.PDelta[t] >= m._target[t] - sum(m.P[i, t] for i in m.devices_idx)
# Defines an upper inequality PDelta >= target - sum P[i,t]. This and the next ensure PDelta >= |target - total_power|.

    def Pdelta_lower_rule(m, t):
        return m.PDelta[t] >= -(m._target[t] - sum(m.P[i, t] for i in m.devices_idx))
# Defines the complementary inequality PDelta >= - (target - total_power).
    model.Pdelta_upper = pyo.Constraint(model.T, rule=Pdelta_upper_rule)
    model.Pdelta_lower = pyo.Constraint(model.T, rule=Pdelta_lower_rule)
    return model
# Instantiates the deviation absolute-value constraints across timesteps.
# Returns the model with added variables and constraints.


def add_device_constraints(model):
    # Function that iterates through devices and appends per-device constraints to the model.
    # constraint lists per device
    model.device_constraints = pyo.ConstraintList()
# Adds a ConstraintList object on the model named device_constraints.
# A ConstraintList lets you add many unnamed constraints programmatically (useful for dynamic per-device constraints).
    for i, dev in enumerate(model._devices):
        state = dev.state
        committed = model._committed_units[i]
# Loops over each device i, extracting its state object and the committed boolean flag (indicating whether that device is allowed to operate in this model).
        if not committed:
            for t in model.T:
                # setting not commited device constraint so that P = 0 for not commited devices
                model.device_constraints.add(model.P[i, t] == 0.0)
            continue
# If the device is not committed (flag False), for all timesteps add constraints P[i,t] == 0.
# This forces which devices are off for the day-ahead schedule.
# After adding those constraints, the loop continues to the next device.
        # add constraints depending on device type
        if is_load_state(state):
            add_load_state_constraints(model, i, state)
        elif is_battery_state(state):
            add_battery_state_constraints(model, i, state)
        elif is_fuel_cell_state(state):
            add_fuel_cell_state_constraints(model, i, state)
        else:
            logging.warning(
                f"Unknown device type at index {i}, no constraints added.")

    return model
# For committed devices, checks their type and calls the corresponding function to add constraints specific to that device type:
# Loads → add_load_state_constraints
# Batteries → add_battery_state_constraints
# Fuel cells → add_fuel_cell_state_constraints
# If the device type isn’t recognized, logs a warning and does not add specialized constraints.
# Returns the model after adding all device constraints.


def add_objective_function(model):
    # Defines the function to add the objective to the model.
    """
    Objective: minimize sum_t c_dev * D[t]  +  sum_{i,t} c_op_i * U[i,t]
    """
# Docstring summarizing the objective: minimize deviation penalty plus operating costs. D[t] are deviation variables PDelta[t], and U[i,t] are absolute operating energy variables Pabs[i,t].
    def obj_rule(m):
        total = 0.0
# Start of objective rule function. Initializes total to accumulate terms.
        for t in m.T:
            # deviation cost for deviation between target and optimized value
            total += m._c_dev * m.PDelta[t]
# Adds deviation penalty c_dev * PDelta[t] for every timestep.
        for i, dev in enumerate(m._devices):
            for t in m.T:
                # operating cost (device specific), going through devices and summing over time
                total += dev.c_op * m.Pabs[i, t]
# Loops through devices and timesteps, adding device-specific operating cost c_op * |P[i,t]| (using Pabs).
        for i, dev in enumerate(m._devices):
            committed = m._committed_units[i]
            if committed:
                total += dev.commitment_cost  # commitment cost for commited units
# Adds a flat commitment cost for devices that are committed (committed True).
# Note: this commitment cost is added once per device across the objective, not per timestep.
        return total
# Returns the assembled scalar objective expression.
    model.obj = pyo.Objective(rule=obj_rule, sense=pyo.minimize)
    return model
# Attaches the objective to the model, telling Pyomo to minimize the obj_rule expression. Returns the model.


def add_load_state_constraints(model, device_index, load_state):
    """
    IdealLoadState:
    - p_min <= P(i,t) <= p_max (with p_max usually = 0)
    """
# Defines constraints specific to IdealLoadState. The docstring explains the constraints: simple bound constraints on power.
    for t in model.T:
        model.device_constraints.add(pyo.inequality(
            float(load_state.p_min), model.P[device_index, t], float(load_state.p_max)))
# For every timestep, adds a bound constraint using pyo.inequality(lower, var, upper). It enforces p_min <= P[i,t] <= p_max. For loads, p_max is typically 0 and p_min is negative (consumption).


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
# Defines battery-specific constraints. Docstring lists:
# power bounds,
# initial and final energy conditions (linking to SOC),
# energy bounds,
# energy evolution equation E_{t+1} = E_t - P_t
# cyclic behavior constraint (final equals specified final SOC)
    for t in model.T:
        model.device_constraints.add(pyo.inequality(float(
            # Power bounds
            battery_state.p_min), model.P[device_index, t], float(battery_state.p_max)))
# For each timestep, adds power bounds p_min <= P[i,t] <= p_max for that battery.
    model.device_constraints.add(model.E[device_index, 0] == float(
        battery_state.size) * float(battery_state.soc))  # initial energy
# Adds constraint initializing battery energy at time 0: E[i,0] = size * soc.
    # final energy (SOC constraint), last value of E is initial value (cyclic behaviour of battery)
    model.device_constraints.add(model.E[device_index, model._n_steps] == float(
        battery_state.size) * float(battery_state.final_soc))
# Sets the final energy at state time n_steps equal to battery size * final_soc. This enforces a SOC requirement at the end of the horizon (e.g., cyclical constraint).
    for t in model.T_state:
        model.device_constraints.add(model.E[device_index, t] <= float(
            battery_state.size))  # energy bounds
        # lower bound 0 is enforced by NonNegativeReals domain
# For each state timestep (0..n_steps), restricts E[i,t] to be ≤ battery capacity size. Lower bound zero is already enforced by the variable domain.
    for t in range(model._n_steps):
        # E in next timestep is E current ts - used power in ts
        model.device_constraints.add(
            model.E[device_index, t+1] == model.E[device_index, t] - model.P[device_index, t])
# For each power timestep t (0..n_steps-1), enforces the energy dynamics: next state's energy equals current state's energy minus the power used in period t.
# Note sign convention: if P is positive when discharging, subtracting P reduces E.


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
# Defines fuel cell constraints. Docstring enumerates:
# power bounds (nonnegative up to p_max),
# initial fuel amount F[i,0] = fuel_amount,
# fuel dynamics similar to battery energy,
# ramp/change limits between consecutive timesteps and between p_prev and first timestep.
    for t in model.T:
        model.device_constraints.add(pyo.inequality(float(
            # adding power bounds
            fuel_cell_state.p_min), model.P[device_index, t], float(fuel_cell_state.p_max)))
# For each timestep, add the inequality p_min <= P[i,t] <= p_max. Note: p_min for fuel cells is typically 0.
    model.device_constraints.add(model.F[device_index, 0] == float(
        fuel_cell_state.fuel_amount))  # initial fuel
# Sets initial fuel amount constraint.
    # dynamics and power bounds
    for t in range(model._n_steps):
        # fuel evolution
        # Fuel amount in next ts is current fuel amount minus Power used in ts
        model.device_constraints.add(
            model.F[device_index, t + 1] == model.F[device_index, t] - model.P[device_index, t])
        # fuel >= 0 is enforced by NonNegativeReals
# For each power timestep, enforces F_{t+1} = F_t - P_t. The sign convention: using power consumes fuel amount numerically equal to P. F nonnegativity comes from variable domain.
    model.device_constraints.add(model.P[device_index, 0] - float(fuel_cell_state.p_prev) <= float(
        # ramp from previous power to first step in absolute value
        fuel_cell_state.change_max))
    model.device_constraints.add(-(model.P[device_index, 0] - float(
        fuel_cell_state.p_prev)) <= float(fuel_cell_state.change_max))  # '''
# Enforces ramp constraint between previous power p_prev and the first modeled timestep P[0]. Because absolute difference is linearized using two inequalities:
# P[0] - p_prev <= change_max
# -(P[0] - p_prev) <= change_max which is equivalent to p_prev - P[0] <= change_max.
# Together they ensure |P[0] - p_prev| <= change_max.
    if model._n_steps > 1:  # ramp between consecutive time steps
        for t in range(0, model._n_steps - 1):
            model.device_constraints.add(
                model.P[device_index, t + 1] - model.P[device_index, t] <= float(fuel_cell_state.change_max))
            model.device_constraints.add(
                model.P[device_index, t] - model.P[device_index, t + 1] <= float(fuel_cell_state.change_max))
# If there’s more than one timestep, enforces ramping between consecutive timesteps by adding two inequalities per pair:
# P[t+1] - P[t] <= change_max
# P[t] - P[t+1] <= change_max
# Together they implement |P[t+1] - P[t]| <= change_max.


def ED_solve(devices, committed_units, target, c_dev):
    # Defines the Economic Dispatch (ED) solver function. ED solves for the optimal continuous power schedule given a set of committed units.
    # ensure no operation here accidentally alters the device objects
    updated_devices = deepcopy(devices)
# Deepcopies the devices list to avoid mutating original device objects during the solve or when reading attributes.
    # build pyomo model
    model = get_pyomo_model(updated_devices, committed_units, target, c_dev)
# Calls get_pyomo_model to build the optimization model for this set of devices and committed units.
    # solve with HiGHS (appsi interface)
    solver = pyo.SolverFactory("appsi_highs")
    result = solver.solve(model, load_solutions=True)
# Creates a solver instance for the appsi_highs backend (a HiGHS solver wrapper integrated with Pyomo through APPSI). Then calls .solve() with load_solutions=True to populate the model with solution values.
    # optional: check solver status
    if (not hasattr(result, "solver")) or (str(result.solver.termination_condition) != "optimal"):
        logging.info(
            f"Solver termination condition: {getattr(result.solver, 'termination_condition', 'Unknown')}")
# Checks whether the solver returned a solver attribute and whether the termination condition is "optimal". If not optimal (or solver info missing), logs the solver termination condition for diagnostics.
    n_dev = len(updated_devices)
    n_steps = len(target)
# Stores counts for devices and timesteps for later use.
    # extract schedules [device][t]
    power_schedules = []
    for i in range(n_dev):
        sched = [pyo.value(model.P[i, t]) for t in model.T]
        # safety: ensure length matches
        if len(sched) != n_steps:
            sched = [pyo.value(model.P[i, t]) for t in range(n_steps)]
        power_schedules.append(sched)
# Extracts the schedule for each device i:
# Builds sched list of model.P[i,t] values for each timestep t in model.T.
# There’s a safety check: if the collected list length does not match n_steps, it constructs the schedule using a range-based fallback.
# This is defensive programming in case model.T differs for some reason.
    # schedule_cost: objective value (deviation + operating cost)
    schedule_cost = pyo.value(model.obj)

    return power_schedules, schedule_cost
# Reads the scalar objective value from the solved model and stores it in schedule_cost.
# This includes deviation cost, operating costs, and commitment costs included in model.obj.
# Returns the power schedules and the corresponding schedule cost to the caller.


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
# Defines the Unit Commitment (UC) solver function.
# UC decides which devices should be committed for the day-ahead schedule.
# The function implements a heuristic that evaluates combinations of committed devices and uses ED_solve to compute their costs.
    n_dev = len(devices)
    counter = 0  # counter for total iterations
    best_commitment = None
    best_cost = None
    i = None  # counter for unchanging iterations
    device_types = []
# Initializes local variables:
# n_dev: number of devices
# counter: counts how many candidate combinations have been tried
# best_commitment: stores best boolean commitment vector found
# best_cost: best cost value found so far
# i: a small counter used to detect lack of improvement (will be used to stop early)
# device_types: to collect device type strings and a ranking value for each device
    for d in devices:
        p_span = d.state.p_max - d.state.p_min
        if p_span == 0:
            return float('inf') # RETURN error
        d.rank_val = d.commitment_cost / p_span + d.c_op
# For each device d, computes:
# p_span: the device’s power span p_max - p_min (range of available power).
# If p_span == 0, meaning the device can’t provide any power, the function returns float('inf') immediately (indicates an error or non-sensical device).
# Computes a rank_val stored as attribute on device (d.rank_val) defined as (commitment_cost / p_span) + c_op.
# This heuristic value balances commitment cost per unit of power span and operating cost, used for ranking devices.
    for dev in devices:
        if is_load_state(dev.state):
            type = "load"
        elif is_battery_state(dev.state):
            type = "battery"
        elif is_fuel_cell_state(dev.state):
            type = "fuel_cell"
        else:
            type = "unknown"
        # getting list with type and rank value
        device_types.append((type, dev.rank_val))
# Classifies each device into a type name string ("load", "battery", "fuel_cell", or "unknown") and appends a tuple (type, rank_val) to device_types.
# This list will be used to aggregate devices by type and create unique representative combinations later.
    all_combinations = [list(combi) for combi in itertools.product(
        [False, True], repeat=n_dev)]
    all_combinations = [row for row in all_combinations if any(row)]
# Generates all 2^n_dev boolean combinations of length n_dev (each combination indicates which devices are committed).
# Then filters out the all-False combination (if any(row)) because at least one device must be committed for a meaningful schedule.
    costs = [cost for _, cost in device_types]
    type_names = [t for t, _ in device_types]
    unique_type_names = list(dict.fromkeys(type_names))
    unique_rows = {}
    target_pmax = max(target)
    target_pmin = min(target)
# Extracts helper lists/values:
# costs: list of rank_val values for each device.
# type_names: list of type name strings for each device.
# unique_type_names: list of unique device type strings while preserving order (using dict.fromkeys trick).
# unique_rows: an empty dict that will map an aggregated count vector (per device type) to a single representative combination (used to deduplicate combinations that only differ by permutation among devices of the same type).
# target_pmax: maximum target power across timesteps.
# target_pmin: minimum target power across timesteps.
    for set in all_combinations:
        total_cost_pu = sum(cost for flag, cost in zip(set, costs) if flag)
        total_pmax = sum(dev.state.p_max for flag,
                         dev in zip(set, devices) if flag)
        total_pmin = sum(dev.state.p_min for flag,
                         dev in zip(set, devices) if flag)
# Iterates through every boolean combination set:
# total_cost_pu: sum of rank_val (cost-per-unit) for devices flagged True in the set. This approximates selection-plus-operational cost per unit (heuristic).
# total_pmax: sum of p_max for the committed devices.
# total_pmin: sum of p_min for committed devices.
        counts = defaultdict(int) # it takes default value 0 for int
        for flag, typ in zip(set, type_names):
            if flag:
                counts[typ] += 1
        # creates "list" with counter of unique type, here unique load, battery,fuel cell -> 3 entries to then
        amount_vec = tuple(counts[t] for t in unique_type_names)
# Counts how many committed devices of each type are in the combination. counts is a defaultdict(int) that increments the count per type if the device flag is True.
# amount_vec is a tuple of counts aligned with unique_type_names.
# This tuple represents the composition of the committed set by device types (e.g., (2 loads, 1 battery, 0 fuel)).
        if amount_vec not in unique_rows:  # if amount_vec not in unique rows yet -> to only include one of the same combinaiton of devices
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
# Deduplicates combinations by only keeping the first encountered combination with a given amount_vec (i.e., the same counts per device type).
# This reduces symmetrical permutations (devices of same type are interchangeable for capacity/aggregate cost approximation).
# Stores metadata for that representative combination in unique_rows, including:
# "set": the boolean commitment vector
# "cost_pu", "pmax", "pmin", and placeholders for later computed fields
    unique_combinations_sorted = sorted(
        unique_rows.values(), key=lambda x: x["cost_pu"])
# Sorts the unique representative combinations by cost_pu (the aggregated rank value).
# This orders combinations from cheapest per-unit to most expensive.

    feasible_combinations = []
    for combination in unique_combinations_sorted:
        pmax = combination["pmax"]
        pmin = combination["pmin"]
        cost_pu = combination["cost_pu"]
        diff_max = target_pmax - pmax
        # target_min might be negative, p_min might be negative (in our cases always <=0)
        diff_min = target_pmin - pmin
# Initializes feasible_combinations list. Then iterates through sorted unique representative combinations and computes:
# diff_max: difference between maximum target and the combination’s max supply (target_pmax - pmax). If negative, combination can supply more than required peak.
# diff_min: difference between minimum target and the combination’s min supply (target_pmin - pmin).
        if diff_max < 0:  # more p providable in combination
            total_cost_max = target_pmax * cost_pu
            feasible = True
        elif diff_max >= 0:  # not enough p in combination
            total_cost_max = pmax * cost_pu + diff_max * c_dev
            feasible = False
# If diff_max < 0, the combination can meet the peak target; total_cost_max approximated as target_pmax * cost_pu and feasible set to True.
# If the combination lacks capacity for peak (diff_max >= 0), a penalty is added: pmax * cost_pu + diff_max * c_dev, and the combination is marked feasible = False.
# This is a coarse feasibility and cost approximation for peak requirement.
        if diff_min < 0:  # less load in combination than needed
            total_cost_min = pmin * cost_pu + diff_min * c_dev
            feasible = False
        elif diff_min >= 0:  # more load in combination than needed
            total_cost_min = target_pmin * cost_pu
            feasible = True
# Similarly handles the minimum target vs. combination minimum capacity:
# If diff_min < 0 (meaning combination’s pmin is greater than target min? Note: because min is often negative, sign logic is a bit tricky in context),
# it computes total_cost_min with penalty and marks infeasible.
# If diff_min >= 0, it sets a simpler cost estimate and marks feasible.
        # sum up absolute values for total cost
        total_cost = abs(total_cost_max) + abs(total_cost_min)
# Combines the two partial costs by summing absolute values of total_cost_max and total_cost_min.
# Using absolute values ensures signs (due to negative targets or pmins) do not cancel and that the metric reflects magnitude of cost.
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
# Updates the combination dict with computed fields and appends it to feasible_combinations.
    # now feasibility check -> feasible and not feasible but cheapest combinations are selected
    feasible = [c for c in feasible_combinations if c["feasible"]]
    infeasible = [c for c in feasible_combinations if not c["feasible"]]
# Splits the combinations into feasible and infeasible based on the coarse feasibility flag computed earlier.
    if feasible:
        min_feasible_cost = min(c["total_cost"] for c in feasible)
    else:
        min_feasible_cost = float('inf')
        print("No feasible combination – all infeasible combos will be kept")
# Finds the minimal total_cost among the feasible combinations. If none are marked feasible, sets min_feasible_cost to infinity and prints a message.
    cheap_infeasible = [
        c for c in infeasible if c["total_cost"] < min_feasible_cost]
    feasible_combinations = feasible + cheap_infeasible
# Selects infeasible combinations that are cheaper than the best feasible one, and keeps them as candidates.
# The union of feasible combos and these cheap infeasible ones forms the filtered feasible_combinations list to be tested precisely by solving ED for each.
    num_feasible = sum(
        1 for entry in feasible_combinations if entry["feasible"])
    num_infeasible = len(feasible_combinations) - num_feasible
    # sort for total cost to check cheapest ones first
    feasible_combinations = sorted(
        feasible_combinations, key=lambda x: x["total_cost"])
# Counts how many of the filtered combos are feasible/infeasible and sorts the filtered list by total_cost (cheapest first).
# This ordering optimizes the search so that ED solves are attempted first on cheaper candidates.
    # going through filtered options
    print("---------------- checking feasible combinations with total length: ",
          len(feasible_combinations))
    for option in feasible_combinations:
        committed_units = option["set"]
        counter += 1
        power_schedules, schedule_cost = ED_solve(
            devices, committed_units, target, c_dev)
# Prints debug info on the number of candidate combinations to check.
# For each candidate option:
# Extracts the boolean commitment vector committed_units.
# Increments the iteration counter.

# Calls ED_solve with that committed_units to compute the precise economic dispatch schedule and objective schedule_cost.
        if best_cost is None or schedule_cost <= best_cost:
            best_cost = schedule_cost
            best_commitment = committed_units
            counter_best = counter
            i = 0
# If this is the first candidate or the schedule cost is better than the current best, update:
# best_cost to schedule_cost
# best_commitment to this committed_units
# counter_best to the current counter (iteration number where improvement happened)
# reset i to 0 (counter of non-improvement iterations)
        elif i > 8:
            print("No improvement within the last 9 iterations")
            break
        else:
            i += 1
# If no improvement was found this iteration:
# If i > 8 (meaning there have been more than 8 consecutive iterations without improvement), print a message and break the loop early.
# This is a heuristic early stop to avoid testing many candidates after no improvement.
# Otherwise increment i to count one more non-improving iteration.
    print("best_cost with feasible solutions", best_cost, "with:", best_commitment,
          "counter was at:", counter_best, "no changes for = ", i, "iterations")
    print("counter", counter)
    return best_commitment
# After iterating candidates (or breaking early), prints debugging info summarizing:
# best objective value found,
# corresponding commitment vector,
# iteration when best was found,
# how many iterations without improvement occurred before stopping,
# total number of ED evaluations attempted.
# Returns best_commitment, the boolean list indicating which devices to commit for the day-ahead schedule.

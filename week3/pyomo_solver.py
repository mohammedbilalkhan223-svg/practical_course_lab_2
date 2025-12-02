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

# import both here to save a headache below
from src.sim_environment.devices.ideal import *

from src.sim_environment.devices.hil import IdealBatteryState as HILBatteryState
from src.sim_environment.devices.hil import IdealLoadState as HILLoadState
from src.sim_environment.devices.hil import IdealFuelCellState as HILFuelState

from src.sim_environment.optimization_problem import SchedulingProblem
import logging
import pyomo.environ as pyo
from copy import deepcopy

#----------------------------------------------
# convenience functions you will likely need
#----------------------------------------------
# decide if a device.state object is for a specific kind of device
def is_battery_state(obj):
    return isinstance(obj, IdealBatteryState) or isinstance(obj, HILBatteryState)

def is_load_state(obj):
    return isinstance(obj, IdealLoadState) or isinstance(obj, HILLoadState)

def is_fuel_cell_state(obj):
    return isinstance(obj, IdealFuelCellState) or isinstance(obj, HILFuelState)

# Generic naming functions to set pyomo model variables
# Add more of these if needed.
# Example use:
#
# c_var = pyo.Var(range(n_steps), domain=pyo.NonNegativeReals, initialize=0)
# setattr(model, cost_name(5), c_var)
#
# This binds a pyomo variable named <cost_5> with content of a pyomo variable
# <c_var> to the <model> object.
#
# You could then subsequently call:
# model.cost_5 to access it.
#
# More likely you will access it via its generic name as well, in this case:
# getattr(model, cost_name(5))

# intended for power values of a device
def power_name(device_number):
    return f"power_{device_number}"

# intended for schedule cost values of a device
def cost_name(device_number):
    return f"cost_{device_number}"

# intended for lists of constraints
# One possible use is to put all constraints specific to one device
# into a corresponding constraint list.
# e.g.
#
# const_list = pyo.ConstraintList()
# setattr(model, const_list_name(device_number), const_list)
#
def const_list_name(device_number):
    return f"constraints_{device_number}"

#----------------------------------------------
# Helper function suggestions to structure the program
# NOTE:
# - This signatures of these functions are not final.
#   You can and probably should alter some of them for your implementation.
#
# - The purpose of these functions is to structure your parsing of the solver inputs
#   into the relevant sources of model constraints.
#
# - Because the number of devices is variable, you need a generic naming scheme to set
#   model properties in your pyomo model.
#----------------------------------------------
def get_pyomo_model(devices, committed_units, target, c_dev):
    # initialize a new pyomo model
    model = pyo.ConcreteModel()

    # add constraints that exist independent of the chosen devices
    model = add_problem_constraints(model)
    
    # add device specific constraints
    model = add_device_constraints(model, devices, committed_units)

    # add objective function
    model = add_objective_function(model)

    return model

def add_problem_constraints(model):
    pass

def add_device_constraints(model, devices, committed_units):
    pass

def add_objective_function(model):
    # NOTE: The inputs to this function rely on the model variables
    # added in the problem and device constraint functions!
    pass

def add_load_state_constraints(load_state, model):
    pass

def add_battery_state_constraints(battery_state, model):
    pass

def add_fuel_cell_state_constraints(fuel_cell_state, model):
    pass
#----------------------------------------------
#----------------------------------------------

"""
Input:
- devices: List of IdealDevice objects
- committed_units: List of {True, False} of same length as devices indicating which are to be committed
- target: the list of target values
- c_dev: the target difference cost factor

Output:
- (power_schedules, schedule_cost)

where:
power_schedules is a list of list of setpoints for each device.
e.g. for 4 devices and a target length of 5 it could be:
[
    [1, 2, 3, 4, 12.4],
    [0, 0, 0, 0, 0],
    [1, 2, 3, 4, 5],
    [-1, -3.5, 0, 0, 0]
]

schedule_cost is the TOTAL cost of the optimization
- i.e. commitment costs + schedule costs
"""
def ED_solve(devices, committed_units, target, c_dev):
    # TODO implement your solver here
    # you can define any helper functions you want within this file
    # The stub functions given here are only a suggestion, you can change
    # them however you like. Just don't change the interface of the 
    # ED_solve and UC_solve functions.

    # ensure no operation here accidentally alters the device objects
    # as these are still used outside of this function
    updated_devices = deepcopy(devices)

    # suggested function for parsing the problem into a pyomo model
    model = get_pyomo_model(updated_devices, committed_units, target, c_dev)    

    # NOTE: commented out because it would error as long as the model is not actually created
    # pyo.SolverFactory("appsi_highs").solve(model)

    # TODO: extract the result schedules from the solved model

    power_schedules = [[0] * len(target) for _ in range(len(updated_devices))]
    schedule_cost = 0
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

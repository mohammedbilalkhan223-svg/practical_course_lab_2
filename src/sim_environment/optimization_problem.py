import numpy as np
import logging

def linear_dev_cost(target, device_powers, c_dev):
# Computes the difference between total device power and target power.
    if any([len(target) != len(dp) for dp in device_powers]):
        logging.error("Mismatch between device power length and target length.")
        return 0
# Ensures all device power profiles match the target timeline.    
    cumulative_cost = 0
# Holds the cost accumulated across timesteps.
    for t in range(len(target)):
        cumulative_cost += abs(target[t] - sum([dp[t] for dp in device_powers])) * c_dev
# At each timestep, compares target vs. total device output.
# Cost = deviation × penalty coefficient.
    return cumulative_cost
# Returns total deviation cost.
class SchedulingProblem:
# Encapsulates the target schedule and devices.
    def __init__(self, target, devices, c_dev, max_rel_rand, dev_cost_function=None):
        self.target = target
        self.devices = devices
        self.c_dev = c_dev
        self.max_rel_rand = max_rel_rand
# target → desired power injection
# devices → list of devices
# c_dev → cost coefficient
# max_rel_rand → randomness cap
# dev_cost_function → optional custom deviation function        
        if dev_cost_function is None:
            dev_cost_function = lambda target, x : linear_dev_cost(target, x, c_dev)
# Uses default deviation cost unless one is provided.      
        self.dev_cost_function = dev_cost_function
# Stores the cost function. 

    def get_total_cost(self, device_powers):
# Computes the total cost of a schedule.
        commitment_costs = sum([x.commitment_cost for x in self.devices if any(x.stepped_powers)])
# Devices pay commitment cost if they were used at least once.
        device_costs = 0
        n_steps = len(self.target)
        total_power = [0] * n_steps
# Initialize counters.
        for t in range(n_steps):
            for i, device in enumerate(self.devices):
                power = device_powers[i][t]
                total_power[t] += power
                device_costs += device.cost(power)
# Computes cost for each device at each timestep.
# Also sums total output power.
        schedule_cost = self.dev_cost_function(self.target, device_powers)
# Computes deviation cost using the selected cost function.
        print(f"Schedule deviation cost was: {schedule_cost}")
        print(f"Device use cost was: {device_costs}")
        return schedule_cost + device_costs + commitment_costs
# Returns total cost.
    def get_cumulative_by_timestep(self, device_powers):
# Provides more detailed per-timestep cost breakdown.
# Computes cost per timestep rather than total.
        device_costs = 0
        n_steps = len(self.target)
        total_power = [0] * n_steps
        device_cost_list = [0] * n_steps
        schedule_cost_list = [0] * n_steps
        total_cost_list = [0] * n_steps
# Initializes arrays.
        for t in range(n_steps):
            for i, device in enumerate(self.devices):
                power = device_powers[i][t]
                total_power[t] += power
                device_costs += device.cost(power)
                device_cost_list[t] += device.cost(power)
# Computes individual and total device costs per timestep.
        schedule_cost = 0
        for t, p in enumerate(total_power):
            this_schedule_cost = self.c_dev * abs(p - self.target[t])
            schedule_cost += this_schedule_cost
            schedule_cost_list[t] = this_schedule_cost
            total_cost_list[t] = this_schedule_cost + device_cost_list[t]
# Computes deviation cost at each timestep and total cost per timestep.
        return total_power, device_cost_list, schedule_cost_list, np.cumsum(total_cost_list)
# Returns total power, per-device costs, deviation cost list, and cumulative cost.

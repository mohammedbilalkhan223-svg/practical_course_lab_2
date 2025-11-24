import numpy as np
import logging

def linear_dev_cost(target, device_powers, c_dev):
    if any([len(target) != len(dp) for dp in device_powers]):
        logging.error("Mismatch between device power length and target length.")
        return 0
    
    cumulative_cost = 0
    for t in range(len(target)):
        cumulative_cost += abs(target[t] - sum([dp[t] for dp in device_powers])) * c_dev

    return cumulative_cost

class SchedulingProblem:
    def __init__(self, target, devices, c_dev, max_rel_rand, dev_cost_function=None):
        self.target = target
        self.devices = devices
        self.c_dev = c_dev
        self.max_rel_rand = max_rel_rand
        
        if dev_cost_function is None:
            dev_cost_function = lambda target, x : linear_dev_cost(target, x, c_dev)
        
        self.dev_cost_function = dev_cost_function
    

    def get_total_cost(self, device_powers):
        commitment_costs = sum([x.commitment_cost for x in self.devices if any(x.stepped_powers)])
        device_costs = 0
        n_steps = len(self.target)
        total_power = [0] * n_steps

        for t in range(n_steps):
            for i, device in enumerate(self.devices):
                power = device_powers[i][t]
                total_power[t] += power
                device_costs += device.cost(power)

        schedule_cost = self.dev_cost_function(self.target, device_powers)
        # print(f"Schedule deviation cost was: {schedule_cost}")
        # print(f"Device use cost was: {device_costs}")
        return schedule_cost + device_costs + commitment_costs

    def get_cumulative_by_timestep(self, device_powers):
        device_costs = 0
        n_steps = len(self.target)
        total_power = [0] * n_steps
        device_cost_list = [0] * n_steps
        schedule_cost_list = [0] * n_steps
        total_cost_list = [0] * n_steps

        for t in range(n_steps):
            for i, device in enumerate(self.devices):
                power = device_powers[i][t]
                total_power[t] += power
                device_costs += device.cost(power)
                device_cost_list[t] += device.cost(power)

        schedule_cost = 0
        for t, p in enumerate(total_power):
            this_schedule_cost = self.c_dev * abs(p - self.target[t])
            schedule_cost += this_schedule_cost
            schedule_cost_list[t] = this_schedule_cost
            total_cost_list[t] = this_schedule_cost + device_cost_list[t]

        return total_power, device_cost_list, schedule_cost_list, np.cumsum(total_cost_list)


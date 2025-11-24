from dataclasses import dataclass
from typing import Dict
from mango import Agent, sender_addr
from mango.messages.codecs import json_serializable
import asyncio
import copy
import random
import numpy as np
from ..sim_environment.optimization_problem import SchedulingProblem
from messages import TargetUpdateMsg, SetDoneMsg, NotifyReadyRequestMsg, NotifyReadyMsg, SetScheduleMsg, StateRequestMsg, StateReplyMsg, SetScheduleReplyMsg

import sys

from ..sim_environment.devices.ideal import *
import logging


class DummyHILObserver(Agent):
    def __init__(self, controller_addresses, step_time_s, problem, seed=None):
        super().__init__()

        self.problem_rng = random.Random(seed)

        self.controller_addresses = controller_addresses
        self.step_time_s = step_time_s
        self.n_steps = len(problem.target)
        self.original_target = copy.deepcopy(problem.target)
        self.randomized_target = copy.deepcopy(problem.target)
        self.max_rel_rand = problem.max_rel_rand
        self.steps_done = 0
        self.done = asyncio.Future()
        self.c_dev = problem.c_dev
        self.rp = None
        self.final_problem = copy.deepcopy(problem)

        self.ready_controllers = {addr: False for addr in self.controller_addresses}
        self.all_controllers_ready = asyncio.Future()

        self.devices = problem.devices
        self.controller_schedules = [[0] * len(problem.target) for _ in range(len(controller_addresses))]

        # some convenience dicts to map corresponding things to each other
        self.con_addr_to_id = {
            addr: i for i, addr in enumerate(controller_addresses)
        }

    def on_register(self):
        pass

    async def shutdown(self):
        await super().shutdown()

    def handle_message(self, content, meta):
        sender = sender_addr(meta)

        if isinstance(content, NotifyReadyMsg):
            self.ready_controllers[sender] = True
            if all(self.ready_controllers.values()):
                self.all_controllers_ready.set_result(True)

        # --------------------------------------------------
        # SetScheduleMsg set things in the HIL and
        # set things in our local device
        # --------------------------------------------------
        if isinstance(content, SetScheduleMsg):
            self.schedule_instant_task(self.handle_set_schedule_msg(content, sender))
            return

        # --------------------------------------------------
        # state request messages read something in the HIL
        # set things in our local device
        # --------------------------------------------------
        if isinstance(content, StateRequestMsg):
            self.schedule_instant_task(self.handle_state_request_msg(content, sender))
            return

    async def handle_set_schedule_msg(self, content, sender):
        index = self.con_addr_to_id[sender]
        self.controller_schedules[index] = content.setpoints

        # reply
        msg = SetScheduleReplyMsg(True)
        await self.send_message(msg, sender)

    async def handle_state_request_msg(self, content, sender):
        index = self.con_addr_to_id[sender]

        # answer with newly updated state
        msg = StateReplyMsg(self.devices[index].state)
        await self.send_message(msg, sender)

    async def set_HIL_power(self, index, p):
        self.devices[index].set_output_power(p)

    #------------------------------------------------
    #------------------------------------------------

    def randomize(self, index):
        og_value = self.original_target[index]
        if og_value == 0:
            return 0

        # random number in (-max_rel, +max_rel)  * og_value
        random_offset = og_value * 2 * (0.5 - self.problem_rng.random()) * self.max_rel_rand
        new_value = og_value + random_offset

        self.randomized_target[index] = new_value
        return new_value

    async def randomize_and_step(self):
        if self.steps_done == self.n_steps:
            return

        # randomize
        new_value = self.randomize(self.steps_done)
        # send information messages
        msg = TargetUpdateMsg(self.steps_done, new_value)
        for addr in self.controller_addresses:
            await self.send_message(msg, addr)

        # wait half step time
        await asyncio.sleep(0.5 * self.step_time_s)

        for i, d in enumerate(self.devices):
            # if controller has not failed, update the power value properly
            # otherwise it remains the same as the old vlaue
            new_p = self.controller_schedules[i][self.steps_done]
            # NOTE: not setting this directly but indirectly from HIL now
            # so the reported power is what was actually set by the HIL device.
            # d.set_output_power(new_p)
            await self.set_HIL_power(i, new_p)
            d.step()

        # termination condition
        self.steps_done += 1
        if self.steps_done == self.n_steps:
            # terminate all other agents
            for addr in self.controller_addresses:
                msg = SetDoneMsg()
                await self.send_message(msg, addr)

            self.done.set_result(True)
        

    async def wait_for_agents_ready(self):
        for addr in self.controller_addresses:
            msg = NotifyReadyRequestMsg()
            await self.send_message(msg, addr)

        await self.all_controllers_ready

    async def start_syncing(self):
        # wait for controller agents to be done with their init
        await self.wait_for_agents_ready()

        # schedule randomization
        self.schedule_periodic_task(
            coroutine_func=self.randomize_and_step, delay=self.step_time_s
        )

    def evaluate(self):
        self.final_problem.target = self.randomized_target
        total_device_costs = sum([x.cumulative_cost for x in self.devices])
        total_diff_cost = self.final_problem.dev_cost_function(self.randomized_target, [x.stepped_powers for x in self.devices])
        total_commitment_costs = sum([x.commitment_cost for x in self.devices if any(x.stepped_powers)])

        total_cost = total_device_costs + total_diff_cost + total_commitment_costs

        print(f"Total deviation cost was: {total_diff_cost}")
        print(f"Total device use cost was: {total_device_costs}")
        print(f"Total commitment cost was: {total_commitment_costs}")
        print(f"Total cost after randomization was: {total_cost}")

        def check_soc_constraint(devices):
            for d in devices:
                if not isinstance(d.state, IdealBatteryState):
                    continue

                if not np.isclose(sum(d.stepped_powers), 0, atol=0.01):
                    print(f"Invalid SOC power diff: {sum(d.stepped_powers)}")
                    return False

            return True

        dev, dp = self.get_devices_and_powers()
        soc_con = check_soc_constraint(dev)

        return total_cost, soc_con

    def get_devices_and_powers(self):
        devices = self.devices
        powers = [d.stepped_powers for d in self.devices]
        return devices, powers
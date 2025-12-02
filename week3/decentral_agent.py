from mango import Agent, sender_addr
from src.sim_environment.optimization_problem import SchedulingProblem
from copy import deepcopy
from pyomo_solver import ED_solve, UC_solve
import asyncio

from messages import (
    TargetUpdateMsg,
    SetDoneMsg,
    SetScheduleMsg,
    NotifyReadyRequestMsg,
    NotifyReadyMsg,
    StateRequestMsg,
    StateReplyMsg
)

class DecentralAgent(Agent):
    def __init__(self, device_address, device, target, c_dev):
        super().__init__()
        self.device_address = device_address
        self.device = device
        self.target = deepcopy(target)
        self.c_dev = c_dev

        # to be set accordingly during the initial scheduling!
        self.committed = True 
        self.device_schedule = [0] * len(target)

        # various futures objects for control flow
        self.init_schedule_done = asyncio.Future()
        self.done = asyncio.Future()
        self.state_request_fut = asyncio.Future()

        # for dumb testing
        self.counter = 0

    def on_register(self):
        pass

    def on_start(self):
        print(f"{self.aid} at: {self.addr}")
        print(f"{self.aid} neighbors: {self.neighbors()}")
        self.schedule_instant_task(self.create_initial_schedule())

    #------------------------------------
    # All about message handling
    #------------------------------------
    def handle_message(self, content, meta):
        # agents are purely reactive so this will result in nothing happening
        sender = sender_addr(meta)

        if isinstance(content, SetDoneMsg):
            self.done.set_result(True)

        if isinstance(content, TargetUpdateMsg):
            self.schedule_instant_task(self.handle_target_update(content, meta))

        if isinstance(content, NotifyReadyRequestMsg):
            self.schedule_instant_task(self.handle_ready_request(sender))

        if isinstance(content, StateReplyMsg):
            self.schedule_instant_task(self.handle_state_reply(content))

    async def handle_target_update(self, content, meta):
        self.target[content.t] = content.value
        remaining_target = self.target[content.t:]
        await self.reschedule(remaining_target, content.t)

    async def handle_ready_request(self, sender):
        await self.init_schedule_done
        await self.update_device_schedule()
        msg = NotifyReadyMsg()
        await self.send_message(msg, sender)

    async def handle_state_reply(self, content):
        self.device.state = content.state
        if not self.state_request_fut.done():
            self.state_request_fut.set_result(True)

    #------------------------------------
    #------------------------------------
    async def update_device_schedule(self):
        msg = SetScheduleMsg(self.device_schedule)
        self.schedule_instant_message(msg, self.device_address)
        

    async def get_device_state(self):
        self.state_request_fut = asyncio.Future()   
        msg = StateRequestMsg()
        await self.send_message(msg, self.device_address)
        await self.state_request_fut

    """
    Call your schedule solver here and save the initial schedule.
    """
    async def create_initial_schedule(self):
        self.committed = await self.solve_UC_decentral()
        self.device_schedule = await self.solve_ED_decentral(self.target)
        self.init_schedule_done.set_result(True)

    async def solve_UC_decentral(self):
        return True

    async def solve_ED_decentral(self, target):
        if not self.committed:
            return [0] * len(target)
        else:
            # something deliberately stupid just to see
            # that each agent sets things on the correct device
            self.counter += 1
            if self.device.state.p_max > 0:
                return [self.device.state.p_max - self.counter] * len(target)
            else:
                return [self.device.state.p_min + self.counter] * len(target)

    """
    Implement your rescheduling logic for the agent here.

    Input:
    - <remaining_target> - the updated schedule with the steps already executed being removed.

    i.e. if the initial schedule was [1, 2, 3, 4, 5], then this function will be called with:
    - [1 + r_1, 2, 3, 4, 5],    t=0
    - [2 + r_2, 3, 4, 5],       t=1
    - [3 + r_3, 4, 5],          t=2
    - [4 + r_4, 5],             t=3
    - [5 + r_5],                t=4
    """
    async def reschedule(self, remaining_target, t):
        await self.get_device_state()
        new_schedule = await self.solve_ED_decentral(remaining_target)
        self.device_schedule[t:] = new_schedule
        await self.update_device_schedule()
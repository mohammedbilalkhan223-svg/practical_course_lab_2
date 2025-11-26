from mango import Agent, sender_addr
from src.sim_environment.optimization_problem import SchedulingProblem
from copy import deepcopy
from pyomo_solver import ED_solve, UC_solve
import asyncio
from itertools import compress

from messages import (
    TargetUpdateMsg,        # for receiving target updates
    SetDoneMsg,             # for simulation control flow
    SetScheduleMsg,         # for setting device schedules
    NotifyReadyRequestMsg,  # for simulation control flow
    NotifyReadyMsg,         # for simulation control flow
    StateRequestMsg,        # ask device for its state
    StateReplyMsg           # receive device state
)

class CentralizedAgent(Agent):
    def __init__(self, device_addresses, devices, target, c_dev):
        super().__init__()
        self.device_addresses = device_addresses

        # local model of the devices
        # REMEMBER: this is used to hold information for the controller
        # THIS IS NOT THE REAL DEVICE STATE!
        #
        # The real device state exists only in the actual device, which can be 
        # communicated with by its device_address!
        self.devices = devices
        self.n_devices = len(self.devices)
        
        self.target = deepcopy(target)
        self.c_dev = c_dev

        self.committed_units = [True for _ in range(self.n_devices)]
        self.device_schedules = [[0] * len(target) for i in range(self.n_devices)]

        # a quick map of the device address to its corresponding index in the various lists here
        self.dev_addr_to_id = {addr: i for i, addr in enumerate(device_addresses)}
        self.updated_schedules = [[0] * len(target) for i in range(self.n_devices)]
        # various futures objects for control flow
        self.init_schedule_done = asyncio.Future()
        self.updated_schedule_done = asyncio.Future()

        self.dones = [asyncio.Future() for i in range(self.n_devices)]
        self.state_request_fut = asyncio.Future()
        self._state_reply_futures: dict[str, asyncio.Future] = {}

    def on_register(self):
        self.schedule_instant_task(self.create_initial_schedule())

    # control flow function, do not change
    async def handle_ready_request(self, sender):
        await self.init_schedule_done
        await self.update_device_schedules(sender)
        msg = NotifyReadyMsg()
        await self.send_message(msg, sender)

    #------------------------------------
    # All about message handling
    #------------------------------------
    def handle_message(self, content, meta):
        sender = sender_addr(meta)

        #-------------------------------
        # Control flow message handling, do not change!
        #-------------------------------
        if isinstance(content, SetDoneMsg):
            self.dones[self.dev_addr_to_id[sender]].set_result(True)

        if isinstance(content, NotifyReadyRequestMsg):
            self.schedule_instant_task(self.handle_ready_request(sender))

        #-------------------------------
        # You can freely make changes below here
        #-------------------------------
        if isinstance(content, TargetUpdateMsg):
            print("TargetUpdateMsg")
            # ignore if coming from more than one proxy device
            # workaround for the central agent being transparent to the observer
            # otherwise this message would be received once for each device, making the handling
            # flow needlessly complicated. This will no longer be an issue in week 3 and 4
            if sender != self.device_addresses[0]:
                return

            # handle this message
            self.schedule_instant_task(self.handle_target_update(content))

        if isinstance(content, StateReplyMsg):
            index = self.dev_addr_to_id[sender]
            #print("old", self.devices[index].state )
            self.devices[index].state = content.state
            #print("new", self.devices[index].state )
            fut = self._state_reply_futures.get(sender)
            if fut and not fut.done():
                fut.set_result(content.state)
                #has to be updated before rescheduling

    async def handle_target_update(self, content):
        print("handle_target_update")
        target_updated = self.target.copy()
        self.target[content.t] = content.value
        if target_updated[content.t]== content.value:
            print("no changes in update")
            return
        else:
            target_updated[content.t] = content.value
            remaining_target = target_updated[content.t:]
            print("sending staterequest message ")
            msg = StateRequestMsg()
            self._state_reply_futures = {addr: asyncio.get_event_loop().create_future() for addr in self.device_addresses}
            for sender in self.device_addresses:
                await self.send_message(msg, sender)
            await asyncio.gather(*self._state_reply_futures.values()) #waits until all states are updated
            print("gathered all state reply futures")
            await self.reschedule(remaining_target, content.t) #start schedule updating
            print("rescheduling done")
            for sender in self.device_addresses: #send the updated schedules to the devices
                await self.update_device_schedules(sender)
            print("updated schedules done")
            await self.updated_schedule_done #updating of schedules is done
        self.updated_schedule_done = asyncio.Future()
        self.state_request_fut = asyncio.Future()
        self._state_reply_futures: dict[str, asyncio.Future] = {}

        pass

    #------------------------------------
    #------------------------------------

    async def update_device_schedules(self, sender):

        #print("schedule", self.device_schedules)
        sender_index = self.device_addresses.index(sender)
        msg = SetScheduleMsg(self.device_schedules[sender_index])
        #print("msg", msg)
        await self.send_message(msg, self.device_addresses[sender_index])
        pass
        

    """
    Call your schedule solver here and save the initial schedule.
    """
    async def create_initial_schedule(self):
        self.committed_units = UC_solve(self.devices, self.target, self.c_dev)
        self.device_schedules = ED_solve(self.devices, self.committed_units, self.target, self.c_dev)[0]
        self.init_schedule_done.set_result(True)


    async def reschedule(self, remaining_target, t):
        print("rescheduling")
        self.updated_schedule = ED_solve(self.devices, self.committed_units, remaining_target, self.c_dev)[0]
        for i, updated in enumerate(self.updated_schedule):
            self.device_schedules[i][t:] = updated #replace old values with new values, starting at t
        self.updated_schedule_done.set_result(True)

        pass
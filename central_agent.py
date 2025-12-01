from mango import Agent, sender_addr
# Imports the Agent base class and a helper for extracting sender addresses from message metadata.
from src.sim_environment.optimization_problem import SchedulingProblem
# Imports the SchedulingProblem class, which describes the target schedule and devices.
from copy import deepcopy
# deepcopy is used to create full independent copies of objects (important so state mutations don’t propagate unintentionally).
from pyomo_solver import ED_solve, UC_solve
# Imports two solver functions:
# - ED_solve: economic dispatch (decide power output given fixed commitments)
# - UC_solve: unit commitment (decide which devices are on/off)
import asyncio
# Used for asynchronous operations, Futures, and message-driven control flow.
from itertools import compress
# compress filters iterables by a boolean selector list, but note: this file does not use it.

from messages import (
    TargetUpdateMsg,        # for receiving target updates
    SetDoneMsg,             # for simulation control flow
    SetScheduleMsg,         # for setting device schedules
    NotifyReadyRequestMsg,  # for simulation control flow
    NotifyReadyMsg,         # for simulation control flow
    StateRequestMsg,        # ask device for its state
    StateReplyMsg           # receive device state
)
# Imports all message classes needed for communication between the central agent and devices.

class CentralizedAgent(Agent):
# Defines the central controller as a Mango Agent.
    def __init__(self, device_addresses, devices, target, c_dev):
# Constructor: receives simulation info such as devices, schedule target, and cost parameters.
        super().__init__()
# Initialize the Agent superclass.
        self.device_addresses = device_addresses
# List of addresses of all physical device agents.
        # local model of the devices
        # REMEMBER: this is used to hold information for the controller
        # THIS IS NOT THE REAL DEVICE STATE!
        #
        # The real device state exists only in the actual device, which can be 
        # communicated with by its device_address!
        self.devices = devices
# Local device models (IdealDevices), not the actual devices.
        self.n_devices = len(self.devices)
# Number of devices.
        self.target = deepcopy(target)
# Copy the target schedule so we can modify it safely.
        self.c_dev = c_dev
# Device change cost parameter used by the solvers.
        self.committed_units = [True for _ in range(self.n_devices)]
# Initial assumption: all units are ON until the UC solver runs.
        self.device_schedules = [[0] * len(target) for i in range(self.n_devices)]
# A 2D list storing the schedule (power output per timestep) for each device.
        # a quick map of the device address to its corresponding index in the various lists here
        self.dev_addr_to_id = {addr: i for i, addr in enumerate(device_addresses)}
# Maps device address → index in self.devices/schedules, allowing quick lookup.
        self.updated_schedules = [[0] * len(target) for i in range(self.n_devices)]
# Temporary buffer for updated schedules during rescheduling.
        # various futures objects for control flow
        self.init_schedule_done = asyncio.Future()
# Will be completed after initial scheduling is created.
        self.updated_schedule_done = asyncio.Future()
# Will be completed after rescheduling finishes.
        self.dones = [asyncio.Future() for i in range(self.n_devices)]
# One Future per device, used when devices notify the central agent that they are done.
        self.state_request_fut = asyncio.Future()
# A placeholder Future for state request workflow (but not heavily used).
        self._state_reply_futures: dict[str, asyncio.Future] = {}
# Dictionary mapping device_address → Future for waiting for state replies.
    def on_register(self):
# Called by Mango when the agent joins the system.
        self.schedule_instant_task(self.create_initial_schedule())
# Starts the initial scheduling asynchronously as soon as the agent is registered.
    # control flow function, do not change
    async def handle_ready_request(self, sender):
# Called when a device asks if the central controller is ready.
        await self.init_schedule_done
# Wait until the initial schedule is complete.
        await self.update_device_schedules(sender)
# Send the calculated schedule to the requesting device.
        msg = NotifyReadyMsg()
# Construct a “ready” message back to the device/simulator.
        await self.send_message(msg, sender)
# Send readiness confirmation.

    #------------------------------------
    # All about message handling
    #------------------------------------
    def handle_message(self, content, meta):
# Main function triggered when any message is received.
        sender = sender_addr(meta)
# Extracts sender agent address from metadata.
        #-------------------------------
        # Control flow message handling, do not change!
        #-------------------------------
        if isinstance(content, SetDoneMsg):
            self.dones[self.dev_addr_to_id[sender]].set_result(True)
# A device has reported completion. Its corresponding Future is resolved.
        if isinstance(content, NotifyReadyRequestMsg):
            self.schedule_instant_task(self.handle_ready_request(sender))
# A device asks if schedules are ready; we asynchronously respond via handle_ready_request().
        #-------------------------------
        # You can freely make changes below here
        #-------------------------------
        if isinstance(content, TargetUpdateMsg):
# Handle updates to the target power schedule.
            # ignore if coming from more than one proxy device
            # workaround for the central agent being transparent to the observer
            # otherwise this message would be received once for each device, making the handling
            # flow needlessly complicated. This will no longer be an issue in week 3 and 4
            if sender != self.device_addresses[0]:
                return
# Only the “first device” is allowed to relay target updates to avoid duplicates.
            # handle this message
            self.schedule_instant_task(self.handle_target_update(content))
# Process the target update asynchronously.
        if isinstance(content, StateReplyMsg):
# A device is replying with its state.
            index = self.dev_addr_to_id[sender]
# Map sender address → device index.
            self.devices[index].state = content.state
# Update the local copy of the device state.
            fut = self._state_reply_futures.get(sender)
# Retrieve the Future waiting on this device’s state.
            if fut and not fut.done():
                fut.set_result(content.state)
                #has to be updated before rescheduling
# Resolve the waiting Future so rescheduling can continue.
    async def handle_target_update(self, content):
# Full handler for when target changes at a specific time t.
        print("handle_target_update")
# Debug print.
        target_updated = self.target.copy()
# Make a shallow copy of the existing target.
        self.target[content.t] = content.value
# Apply the change to the stored target.
        if target_updated[content.t]== content.value:
            print("no changes in update")
            return
# If nothing changed, exit early.
        else:
            target_updated[content.t] = content.value
            remaining_target = target_updated[content.t:]
# Update the local copy and extract only the remaining target from time t onward.
            msg = StateRequestMsg()
# Construct a message to request device states.
            self._state_reply_futures = {addr: asyncio.get_event_loop().create_future() for addr in self.device_addresses}
# Create one Future per device for waiting for state responses.
            for sender in self.device_addresses:
                await self.send_message(msg, sender)
# Send a state request to every device.
            await asyncio.gather(*self._state_reply_futures.values()) #waits until all states are updated
# Wait for all devices to reply.
            await self.reschedule(remaining_target, content.t) #start schedule updating
# Call rescheduling for the new target from time t onward.
            for sender in self.device_addresses: #send the updated schedules to the devices
                await self.update_device_schedules(sender)
# Transmit updated schedules to each device.
            print("updated schedules done")
            await self.updated_schedule_done #updating of schedules is done
# Wait until the reschedule() function signals completion.
        self.updated_schedule_done = asyncio.Future()
# Reset the Future for next update.
        self.state_request_fut = asyncio.Future()
# Reset state request Future.
        self._state_reply_futures: dict[str, asyncio.Future] = {}
# Clear the reply Future dictionary.
        pass

    #------------------------------------
    #------------------------------------

    async def update_device_schedules(self, sender):
# Sends a device its assigned schedule.
        sender_index = self.device_addresses.index(sender)
# Look up device index by its address.
        msg = SetScheduleMsg(self.device_schedules[sender_index])
# Create a message containing the device’s assigned power schedule.
        await self.send_message(msg, self.device_addresses[sender_index])
# Send the schedule to the correct device agent.
        pass
        

    """
    Call your schedule solver here and save the initial schedule.
    """
    async def create_initial_schedule(self):
# Called once at startup to build the initial control plan.
        self.committed_units = UC_solve(self.devices, self.target, self.c_dev)
# First: run Unit Commitment to decide which devices are ON.
        #self.committed_units = [True for _ in range(self.n_devices)]
# (Commented) Option to bypass UC.
        self.device_schedules = ED_solve(self.devices, self.committed_units, self.target, self.c_dev)[0]
# Second: run Economic Dispatch to compute actual power schedule.
        self.init_schedule_done.set_result(True)
# Signal that the initial schedule is ready.


    async def reschedule(self, remaining_target, t):
# Recomputes schedule from time t onward after target changes.
        print("rescheduling")
        self.updated_schedule = ED_solve(self.devices, self.committed_units, remaining_target, self.c_dev)[0]
# Solve ED again, but only for the remaining target window.
        for i, updated in enumerate(self.updated_schedule):
            self.device_schedules[i][t:] = updated #replace old values with new values, starting at t
# Insert the updated partial schedule back into the full schedule.
        self.updated_schedule_done.set_result(True)
# Signal completion of rescheduling.
        pass
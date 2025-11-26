from mango import Agent, sender_addr
from copy import deepcopy
from pyomo_solver import ED_solve, UC_solve
import asyncio

from messages import (
    TargetUpdateMsg,
    SetDoneMsg,
    NotifyReadyRequestMsg,
    NotifyReadyMsg,
    SetScheduleMsg,
)


class CentralizedAgent(Agent):
    def __init__(self, device_addresses, devices, target, c_dev):
        super().__init__()
        self.device_addresses = device_addresses
        self.devices = devices
        self.n_devices = len(self.devices)
        self.target = deepcopy(target)
        self.c_dev = c_dev

        # Commitment status: True = committed, False = not committed
        self.committed_units = [True for _ in range(self.n_devices)]
        # Store device schedules: [device][t]
        self.device_schedules = [[0] * len(target) for _ in range(self.n_devices)]

        # Map device address to index
        self.dev_addr_to_id = {
            addr: i for i, addr in enumerate(device_addresses)
        }

        # Futures for control flow
        self.init_schedule_done = asyncio.Future()
        self.dones = [asyncio.Future() for _ in range(self.n_devices)]
        self.state_request_fut = asyncio.Future()

    def on_register(self):
        # Start creating initial schedule
        self.schedule_instant_task(self.create_initial_schedule())

    async def handle_ready_request(self, sender):
        # Wait for initial schedule to be ready
        await self.init_schedule_done
        # Update device schedules
        await self.update_device_schedules()
        # Send confirmation
        msg = NotifyReadyMsg()
        await self.send_message(msg, sender)

    def handle_message(self, content, meta):
        sender = sender_addr(meta)

        # Control flow: device signals it's done
        if isinstance(content, SetDoneMsg):
            self.dones[self.dev_addr_to_id[sender]].set_result(True)

        # Control flow: device requests readiness
        if isinstance(content, NotifyReadyRequestMsg):
            self.schedule_instant_task(self.handle_ready_request(sender))

        # Handle target update from Observer
        if isinstance(content, TargetUpdateMsg):
            # Ignore if not from first device (workaround for transparency)
            if sender != self.device_addresses[0]:
                return
            # Schedule redispatch
            self.schedule_instant_task(
                self.handle_target_update(content, meta)
            )

    async def handle_target_update(self, content, meta):
        # Extract new target value at time t
        t = content.t
        new_value = content.value

        # Update the target at time t
        self.target[t] = new_value

        # Re-optimize: use ED_solve with current commitment
        # Note: We do not re-run UC unless devices are added/removed
        # For now, assume commitment status remains the same
        power_schedules, cost = ED_solve(
            self.devices,
            self.committed_units,
            self.target,
            self.c_dev
        )

        # Update local schedules
        self.device_schedules = power_schedules

        # Send new schedules to devices
        await self.update_device_schedules()

        # Optional: log cost
        print(f"Redispatch at t={t}: New cost = {cost:.2f}")

    async def update_device_schedules(self):
        # Send new schedules to all devices
        for i in range(self.n_devices):
            msg = SetScheduleMsg(self.device_schedules[i])
            await self.send_message(msg, self.device_addresses[i])

    async def create_initial_schedule(self):
        # Step 1: Solve Unit Commitment (UC)
        self.committed_units = UC_solve(self.devices, self.target, self.c_dev)

        # Step 2: Solve Economic Dispatch (ED)
        power_schedules, cost = ED_solve(
            self.devices,
            self.committed_units,
            self.target,
            self.c_dev
        )

        # Store schedules
        self.device_schedules = power_schedules

        # Mark initialization as done
        self.init_schedule_done.set_result(True)

        # Optional: log initial cost
        print(f"Initial schedule created. Total cost: {cost:.2f}")

    async def reschedule(self, remaining_target, t):
        # This method is not used in this implementation
        # But if needed, it could be used for rolling horizon
        # For now, we handle redispatch via handle_target_update
        pass
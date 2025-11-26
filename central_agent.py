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

        self.committed_units = [True for _ in range(self.n_devices)]
        self.device_schedules = [[0] * len(target) for _ in range(self.n_devices)]

        self.dev_addr_to_id = {
            addr: i for i, addr in enumerate(device_addresses)
        }

        self.init_schedule_done = asyncio.Future()
        self.dones = [asyncio.Future() for _ in range(self.n_devices)]
        self.state_request_fut = asyncio.Future()

    def on_register(self):
        self.schedule_instant_task(self.create_initial_schedule())

    async def handle_ready_request(self, sender):
        await self.init_schedule_done
        await self.update_device_schedules()
        msg = NotifyReadyMsg()
        await self.send_message(msg, sender)

    def handle_message(self, content, meta):
        sender = sender_addr(meta)

        if isinstance(content, SetDoneMsg):
            self.dones[self.dev_addr_to_id[sender]].set_result(True)

        if isinstance(content, NotifyReadyRequestMsg):
            self.schedule_instant_task(self.handle_ready_request(sender))

        if isinstance(content, TargetUpdateMsg):
            if sender != self.device_addresses[0]:
                return
            self.schedule_instant_task(self.handle_target_update(content, meta))

    async def handle_target_update(self, content, meta):
        t = content.t
        new_value = content.value
        self.target[t] = new_value

        power_schedules, cost = ED_solve(
            self.devices,
            self.committed_units,
            self.target,
            self.c_dev
        )

        self.device_schedules = power_schedules
        await self.update_device_schedules()

        print(f"Redispatch at t={t}: New cost = {cost:.2f}")

    async def update_device_schedules(self):
        for i in range(self.n_devices):  # ✅ Fixed
            msg = SetScheduleMsg(self.device_schedules[i])
            await self.send_message(msg, self.device_addresses[i])

    async def create_initial_schedule(self):
        self.committed_units = UC_solve(self.devices, self.target, self.c_dev)
        power_schedules, cost = ED_solve(
            self.devices,
            self.committed_units,
            self.target,
            self.c_dev
        )
        self.device_schedules = power_schedules
        self.init_schedule_done.set_result(True)
        print(f"Initial schedule created. Total cost: {cost:.2f}")

    async def reschedule(self, remaining_target, t):
        pass
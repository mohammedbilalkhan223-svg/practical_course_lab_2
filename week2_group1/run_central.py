import asyncio
import random
import multiprocessing
from mango import AgentAddress, create_tcp_container, activate

import sys
from week2_group1.src.sim_environment.devices.ideal import *
from week2_group1.src.agent_setups.HIL_observer import DummyHILObserver as HILObserver

# -------------------
from central_agent import CentralizedAgent
from week2_group1.src.agent_setups.proxy_agent import ProxyAgent
from week2_group1.scenarios.test_scenarios import get_test_scenarios
# -------------------

from messages import SCENARIO_CODEC

# use cmd scenario number if it exists
SCENARIO_NR = 0
RNG_SEED = 0
if len(sys.argv) > 1:
    SCENARIO_NR = int(sys.argv[1])

if len(sys.argv) > 2:
    RNG_SEED = int(sys.argv[2])

# change these lines to switch between the hil_scenarios and test_scenarios
# beware that they have different formats and a lot of the parameters in the
# hil_scenarios.py file are unused here (only relevant in lab 3).
def get_scenario():
    #---------------------------------------
    # step_time_s, _, problem, _, _, _  = get_hil_scenarios()[SCENARIO_NR]
    step_time_s = 0.2
    problem = get_test_scenarios()[SCENARIO_NR]

    return step_time_s, problem
    #---------------------------------------

HOST = "127.0.0.1"
OBS_PORT = 5555
CON_PORT = 5557

OBS_NAME = "Observer"
OBS_ADDR = AgentAddress((HOST, OBS_PORT), OBS_NAME)

def con_name(i):
    return f"con_{i}"


def con_addr(i):
    return AgentAddress((HOST, CON_PORT), con_name(i))


async def main():
    random.seed(1)
    step_time_s, problem = get_scenario()    

    c_proc = make_controllers_process()
    c_proc.start()

    await run_observer(step_time_s, problem)

    # kill in case join does not work for some reason
    # to ensure termination
    c_proc.join(timeout=1)
    c_proc.kill()

    print("done")

def make_controllers_process():
    process = multiprocessing.Process(target=run_controllers)
    return process


def run_controllers():
    async def controller_main():
        step_time_s, problem = get_scenario()
        devices = problem.devices
        n_agents = len(devices)
        container = create_tcp_container(addr=(HOST, CON_PORT), codec=SCENARIO_CODEC)

        device_addresses = [AgentAddress((HOST, CON_PORT), con_name(i)) for i in range(n_agents)]
        
        central_agent = CentralizedAgent(device_addresses, devices, problem.target, problem.c_dev)
        container.register(central_agent, suggested_aid="central_agent")

        # make proxy agents
        for i in range(n_agents):
            a = ProxyAgent(OBS_ADDR, central_agent.addr)
            container.register(a, suggested_aid=con_name(i))  

        async with activate(container) as c1:
            await central_agent.dones[0]
            for done_future in central_agent.dones:
                await done_future

    asyncio.run(controller_main())

async def run_observer(step_time_s, problem):
    container = create_tcp_container(addr=(HOST, OBS_PORT), codec=SCENARIO_CODEC)
    controller_addresses = [con_addr(i) for i in range(len(problem.devices))]
    # (self, device_addresses, controller_addresses, step_time_s, problem)
    obs = HILObserver(controller_addresses, step_time_s, problem, RNG_SEED)
    container.register(obs, suggested_aid=OBS_NAME)

    async with activate(container) as c1:
        await asyncio.sleep(1)
        await obs.start_syncing()
        await obs.done

    obs.evaluate()
    plot_results(obs)

def plot_results(obs):
    import matplotlib.pyplot as plt

    devices, device_powers = obs.get_devices_and_powers()
    total_power, device_cost_list, schedule_cost_list, cumulative_cost = (
        obs.final_problem.get_cumulative_by_timestep(device_powers)
    )

    x = list(range(len(total_power)))
    plt.plot(x, obs.final_problem.target, label = "target power")
    plt.plot(x, total_power, label ="total power")
    plt.plot(x,  device_cost_list, label = "device costs")
    plt.plot(x, schedule_cost_list, label = "target costs")
    plt.plot(x, cumulative_cost, label = "total costs")
    plt.legend()
    plt.show()

    plt.cla()
    plt.plot(x, obs.final_problem.target, label = "target power")
    print(f"target: {obs.final_problem.target}")
    for i, schedule in enumerate(device_powers):
        added_string = ""
        if isinstance(devices[i].state, IdealBatteryState):
            added_string = "bat"
        if isinstance(devices[i].state, IdealLoadState):
            added_string = "load"
        if isinstance(devices[i].state, IdealFuelCellState):
            added_string = "fuel"

        l = f"device_{i}_{added_string}"
        plt.plot(x, schedule, label = l)
        print(f"schedule: {l} - {schedule}")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    asyncio.run(main())

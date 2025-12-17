import logging
import asyncio
import random
import multiprocessing
from mango import run_with_tcp, AgentAddress, create_tcp_container, activate, custom_topology, per_node

import sys
from src.sim_environment.devices.ideal import *
from src.agent_setups.HIL_observer import DummyHILObserver as HILObserver
from src.sim_environment.optimization_problem import *

# -------------------
from decentral_agent import DecentralAgent
from scenarios.decentral_scenarios import get_test_scenarios
from messages import SCENARIO_CODEC, SetDoneMsg
# -------------------

import networkx as nx
import os
import json

# use cmd scenario number if it exists
SCENARIO_NR = 0
RNG_SEED = 0
STEP_TIME_S = 0.2
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
    problem, topology = get_test_scenarios()[SCENARIO_NR]

    return STEP_TIME_S, problem, topology
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
    step_time_s, problem, topology = get_scenario()    
    await run_containers(step_time_s, problem, topology)
    print("done")

async def run_containers(step_time_s, problem, topology):
    devices = problem.devices
    n_agents = len(devices)

    # observer and its container
    obs_container = create_tcp_container(addr=(HOST, OBS_PORT), codec=SCENARIO_CODEC)
    controller_addresses = [con_addr(i) for i in range(len(problem.devices))]
    # (self, device_addresses, controller_addresses, step_time_s, problem)
    obs = HILObserver(controller_addresses, step_time_s, problem, RNG_SEED)
    obs_container.register(obs, suggested_aid=OBS_NAME)

    # controllers and their container
    con_container = create_tcp_container(addr=(HOST, CON_PORT), codec=SCENARIO_CODEC)
    device_addresses = [OBS_ADDR for i in range(n_agents)]
    agents = []
    for i, addr in enumerate(device_addresses):
        agent = DecentralAgent(addr, devices[i], problem.target, problem.c_dev)
        con_container.register(agent, suggested_aid=con_name(i))
        agents.append(agent)

    # set agent topology
    for i, node in enumerate(per_node(custom_topology(topology))):
        node.add(agents[i])


    async with activate(obs_container, con_container) as c1:
        await asyncio.gather(obs.start_syncing(), obs.done, *[a.done for a in agents])
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
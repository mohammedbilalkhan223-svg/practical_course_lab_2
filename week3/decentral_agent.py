from mango import Agent, sender_addr
from matplotlib import pyplot as plt

from src.sim_environment.optimization_problem import SchedulingProblem
from copy import deepcopy
from pyomo_solver import ED_solve, UC_solve, is_battery_state, is_load_state, is_fuel_cell_state
import asyncio
import numpy as np
from messages import *
import pyswarms as ps
import time
'''Basic idea:
- every agent gets own routing list and knows type of device
- every agent creates random initial feasible schedule -> share schedule with others -> everyone optimizes own schedule & shares 
- every agent saves personal best cost and schedule, finds global best schedule and cost (so far)
- get some trade off between personal best and global best as new schedules of others, reoptimize own schedule 
- reanalyse personal best and global best, if new global best, share with all 
- get some trade off between personal best and global best as new schedules of others, reoptimize own schedule 
'''
class DecentralAgent(Agent):
    def __init__(self, device_address, device, target, c_dev):
        super().__init__()
        self.device_address = device_address  # device address of the device agent is sitting on
        self.device = device  # device object to access device information
        self.all_devices = []
        self.target = deepcopy(target)
        self.c_dev = c_dev

        # to be set accordingly during the initial scheduling!
        self.committed = [True]
        self.device_schedule = [0] * len(target)

        # various futures objects for control flow
        self.init_schedule_done = asyncio.Future()
        self.done = asyncio.Future()
        self.state_request_fut = asyncio.Future()
        self.all_replies_arrived = None

        # for dumb testing
        self.device_replies = {}
        self.own_device_state = None
        self.counter = 0
        self.registered_for= {}
        self.parent = None
        self.agent_info = {}
        self.agent_routing = None
        self.neighbor_aid = None
        self.all_device_schedules = None
        self.state_update_future = asyncio.Future()
        self.get_device_future = asyncio.Future()
        self.identification_forwarded = asyncio.Future()
        self.schedule_updated = asyncio.Future()
        self.evaluation_done = asyncio.Future()
        self.dev_c_op = {}
        self.GB_schedules = {}
        self.GB_cost = float('inf')
        self.proposed_schedules = None
        self.PSO_process = asyncio.Future()
        self.initial_evaluation_done = asyncio.Future()
        self.PSO_counter = 0
        self.GB_cost_it = []
        self.received_PSO_replies = []
        self.all_PSO_future = asyncio.Future()
        self.NoUpdateCounter = 0
        self.all_GB_schedules = {}
        self.all_GB_costs = {}
        self.best_schedule_found = asyncio.Future()
        self.total_schedule_start_time = None

    def on_register(self):
        pass

    def on_start(self):
        print(f"{self.aid} at: {self.addr}")
        print(f"{self.aid} neighbors: {self.neighbors()}")
        self.schedule_instant_task(self.identify_agents())
        self.schedule_instant_task(self.create_initial_schedule())

    # ------------------------------------
    # All about message handling
    # ------------------------------------
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

        if isinstance(content, IdentifyAgentsMsg):
            self.schedule_instant_task(self.handle_identify_agents(content, sender, meta))

        if isinstance(content, NewGlobalBestMsg):
            self.schedule_instant_task(self.handle_NewGlobalBestMsg(content, sender))

        if isinstance(content, InitialScheduleMsg):
            self.schedule_instant_task(self.handle_InitialScheduleMsg(content, sender))


    ### Helper function
    async def forward_msg(self, content, sender):
        other_neighbors = [n for n in self.neighbors() if n != sender]  # forward message to other neighbors
        if other_neighbors:
            for neighbor in other_neighbors:  # send message only to others, not to sender not notify
                #print(self.aid, "forwarding msg to", neighbor)
                await self.send_message(content, neighbor)
        else:
            #print(self.aid, "no other neighbors than parent, sending it to parent")
            await self.send_message(content, sender)  # start sending it back to sender

    async def evaluate_schedule(self, schedules):
        print(self.aid, f"runtime counter {self.PSO_counter}")
        self.received_PSO_replies = []
        self.all_PSO_future = asyncio.Future()
        await self.PSO_process #set when PSO process is done
        print(self.aid, "evaluate schedule and PSO Process future is true")
        c_total = await self.cost_fcn(schedules) #calculating cost again bec. no cost for constraint violations
        if c_total <= self.GB_cost:
            #print(self.aid, f"new GB with {self.GB_cost} to {c_total}")
            self.GB_cost_it.append(c_total) #appending new GB cost
            self.all_GB_costs[self.aid] = c_total
            self.all_GB_schedules[self.aid] = schedules
            #self.GB_schedules = schedules
            #self.GB_cost_it.append(c_total)
            #await self.update_device_schedule()
            msg = NewGlobalBestMsg(self.aid, GB_cost=self.all_GB_costs[self.aid], GB_schedules=self.all_GB_schedules[self.aid])
            print(self.aid, "Informing neighbors about new GB")
            for neighbor in self.neighbors():
                await self.send_message(msg, neighbor)
        elif c_total >= self.GB_cost: #if nothing changed, send that no new GB found
            msg = NewGlobalBestMsg(self.aid, GB_cost=self.GB_cost, GB_schedules=self.GB_schedules) #just sending old GB values
            for neighbor in self.neighbors():
                await self.send_message(msg, neighbor)
        #await self.find_new_GB()
        #self.evaluation_done.set_result(True)

    async def find_new_GB(self):
        #await self.all_PSO_future
        print(self.aid, "PSO Process future is true in evaluate_schedule, now we find cheapest schedule")
        best_aid = min(self.all_GB_costs, key=self.all_GB_costs.get)
        best_cost = self.all_GB_costs[best_aid]
        best_schedule = self.all_GB_schedules[best_aid]
        print(self.aid, f"GB cost {self.GB_cost}")
        print(self.aid, f"best_cost = {best_cost}, best_schedule = {best_schedule}")
        if float(best_cost) >= float(self.GB_cost):
            print(self.aid, "No cheaper schedule found")
            if not self.best_schedule_found.done():
                self.best_schedule_found.set_result(True)
        elif float(best_cost) < float(self.GB_cost):
            print(self.aid, "cheaper schedule found, now starting PSO")
            self.GB_cost = best_cost
            self.GB_schedules = best_schedule
            self.PSO_process = asyncio.Future()
            await self.solve_PSO_decentral()

        #self.GB_schedules = self.all_GB_schedules
        #self.GB_cost = self.all_GB_costs

    async def cost_fcn(self, schedules):
        P_tot = [sum(schedules[aid][0][t] for aid in schedules) for t in range(len(self.target))]
        P_dev = [abs(P_tot[t] - self.target[t]) for t in range(len(self.target))]
        sum_c_op = sum(sum(abs(P) * self.dev_c_op[aid] for P in schedules[aid][0]) for aid in schedules)
        c_total = (sum(P_dev) * self.c_dev) + sum_c_op

        return c_total

    ### Msg Handling

    async def handle_target_update(self, content, meta):
        self.best_schedule_found = asyncio.Future()
        if self.target[content.t] != content.value:
            self.target[content.t] = content.value
            remaining_target = self.target[content.t:]
            await self.reschedule(remaining_target, content.t)
        else:
            print("No update in target")


    async def handle_ready_request(self, sender):
        await self.init_schedule_done
        await self.update_device_schedule()
        msg = NotifyReadyMsg()
        await self.send_message(msg, sender)

    async def handle_state_reply(self, content):
        self.device.state = content.state
        if not self.state_request_fut.done():
            self.state_request_fut.set_result(True)

    async def handle_identify_agents(self, content, sender, meta):
        '''IDEA:
        every agent sends out message to neighbors which at some point returns (multiple times)
        if I am not sender, I register myself (with my aid and my parent) to the dict and then forward the message
        if I am the original sender, I check if the new agent list has more agents than my already received one '''

        agents = content.agents
        original_sender = content.sender_aid

        if not original_sender in self.registered_for: #check if already registered to the sender
            if content.sender_aid != self.aid: #check that I was not the original sender
                if self.aid not in agents:  # if I am not in agent list already:
                    #print(self.aid, "registering to agents dict")
                    agents[self.aid] = {"parent": sender}  # add aid and parent to dict
                    self.registered_for[original_sender] = {"access_via": sender}  # writing sender_aid to registered_to list
                    await self.forward_msg(content, sender)
                else:
                    print(self.aid, "something went wrong, I do nto remember registering")

        elif original_sender in self.registered_for: #I already registered once to this agent and forwarded it afterwards
                #print(self.aid, "sending msg to direction of original sender")
                parent = self.registered_for[original_sender]["access_via"]
                await self.send_message(content, parent)

        if content.sender_aid == self.aid:  # if I am original sender and don't forward the message anymore
            #print(self.aid, "I got my message back")
            for aid, data in content.agents.items():  # if there are agents yet unkown in received list, append them
                if aid not in self.agent_info:
                    self.agent_info[aid] = data
            if len(self.agent_info) < len(self.registered_for):
                print("I registered to more than I received and wait a bit longer")
            print(self.aid, "the agents I know are:", len(self.agent_info), self.agent_info)
            await asyncio.sleep(3)

    async def handle_InitialScheduleMsg(self, content, sender):
        self.dev_c_op[content.sender_aid] = content.c_op
        if content.sender_aid not in self.GB_schedules: #if sender_aid of schedule not in PB schedueles yet
            self.GB_schedules[content.sender_aid] = content.schedule  # saving received schedule as current PB
            await self.forward_msg(content, sender) #forward the msg
            if len(self.GB_schedules) == (len(self.agent_info)+1):
                #print("Received as many schedueles as I know agents, setting future")
                self.schedule_updated.set_result(True) #all initial schedules arrived
                #print(self.aid, "schedule updated future set")
        else:
            pass

    async def identify_agents(self):
        # send message to all agents to find all agents
        agents = {}
        msg = IdentifyAgentsMsg(self.aid, agents=agents) #sending msg with own aid and empty agents list
        for neighbor in self.neighbors():  # send message with highest id to all neighbors
            print(self.aid, "sending message to: ", neighbor)
            await self.send_message(msg, neighbor)


    async def handle_NewGlobalBestMsg(self, content, sender):
        sender_aid = content.sender_aid
        #print(self.aid, f"sender_aid = {sender_aid}, received_PSO_replies = {self.received_PSO_replies}")
        #print(self.aid, f"all_GB_schedules = {self.all_GB_schedules}")
        if sender_aid not in self.received_PSO_replies:
            self.all_PSO_future = asyncio.Future()
            self.received_PSO_replies.append(sender_aid)
            self.all_GB_schedules[sender_aid] =  content.GB_schedules
            self.all_GB_costs[sender_aid] = content.GB_cost
            await self.forward_msg(content, sender)

            if len(self.received_PSO_replies) == len(self.agent_info):
                print(self.aid, f"received PSO relies: {self.received_PSO_replies}")
                print(self.aid, "setting all_PSO_future to done")
                print(self.aid, f"all_GB_cost: {self.all_GB_costs}")
                print(self.aid, f"all_GB_schedules: {self.all_GB_schedules}")
                await self.find_new_GB()
                # find lowest costs
                # check if lower than old GB
                #if not self.all_PSO_future.done():
                    #print(self.aid, "all PSO relies done")
                    #self.all_PSO_future.set_result(True)
        elif sender_aid in self.received_PSO_replies and content.GB_schedules == self.all_GB_schedules[sender_aid]:
            pass
        elif sender_aid in self.received_PSO_replies and content.GB_schedules != self.all_GB_schedules[sender_aid]:
            pass



    # ------------------------------------
    # ------------------------------------

    async def update_device_schedule(self):
        # for updating my own schedule
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
    async def set_starting_schedule(self, target_length):
        await self.get_device_state()
        if is_battery_state(self.device.state):
                self.own_device_state = "Battery"
                #print(self.aid, self.device.state)
        elif is_fuel_cell_state(self.device.state):
                self.own_device_state = "FuelCell"
                #print(self.aid, self.device.state)

        elif is_load_state(self.device.state):
            self.own_device_state = "Load"
            #print(self.aid, self.device.state)
        devices = [self.device]
        num_agents = len(self.agent_info)+1 #+1 because own agent not in list
        agent_target = [x / num_agents  * np.random.uniform(0.8, 1.2) for x in self.target]
        c_dev = self.c_dev
        start_time = time.perf_counter()
        init_schedule, init_cost = ED_solve(devices, self.committed, agent_target, c_dev) #to ensure initial schedule meets all constraints we use ED solve
        end_time = time.perf_counter()
        solver_runtime = end_time - start_time
        print(f"{self.aid} ED_solve runtime: {solver_runtime:.4f} seconds")
        #init_schedule = [0.5 * self.device.state.p_max for _ in range(len(self.target))]
        self.device_schedule = init_schedule
        self.GB_schedules[self.aid] = init_schedule
        self.dev_c_op[self.aid] = self.device.c_op
        msg = InitialScheduleMsg(init_schedule, self.dev_c_op[self.aid], sender_aid=self.aid)
        for neighbor in self.neighbors():
            await self.send_message(msg, neighbor)
        print(self.aid, "waiting for schedule updated future")
        await self.schedule_updated #wait until all initial schedules arrived
        print(self.aid, "running PSO")
        self.PSO_process = asyncio.Future()
        await self.solve_PSO_decentral()
        #self.init_schedule_done.set_result(True)
        #self.initial_evaluation_done.set_result(True)


    async def create_initial_schedule(self):
        self.total_schedule_start_time = time.perf_counter()
        await asyncio.sleep(3)
        target_length = len(self.target)
        await self.set_starting_schedule(target_length) #creates bit random but working initial schedule
        await self.best_schedule_found
        print(self.aid, "create initial schedule, updating device schedule")
        print(self.aid, "device schedule: ", self.GB_schedules[self.aid])
        self.device_schedule = self.GB_schedules[self.aid][0]
        await self.update_device_schedule()
        if not self.init_schedule_done.done():
            self.init_schedule_done.set_result(True)
        total_runtime = time.perf_counter() - self.total_schedule_start_time
        print(f"{self.aid} Total scheduling runtime: {total_runtime:.4f} seconds")

    def constraint_cost(self, schedules):
        constraint_penalty = 0.0
        penalization = 1e6
        schedule = schedules[self.aid][0]
        if self.own_device_state == "Battery":
            '''- P_min <= P(i, t) <= P_max -> via bounds
            '''
            E0 = self.device.state.size * self.device.state.soc #E of device; E(i, 0) = size * soc
            Et = E0
            for t, Pt in enumerate(schedule):
                Et1 = Et-Pt #current energy in each step ; E(i, t + 1) = E(i, t) - P(i, t)
                if Et < 0 or Et > self.device.state.size: # 0 <= E(i, t) <= size
                    constraint_penalty += penalization #adding fixed penalization for violation
                E1t = Et #E from current time step saved for next round as E(t-1) -> important for E_T evaluation in next if ...
                Et = Et1 #setting current E for next rounds previous
                soc = Et / self.device.state.size
                if soc <0 or soc>1: #making sure soc stays within bounds
                    constraint_penalty += penalization
            if E1t != self.device.state.size * self.device.state.final_soc: #E(i, T) = size * final_soc ;
                constraint_penalty += penalization

            if Et != self.device.state.size * self.device.state.final_soc: #E(i, T + 1) = E(t=0)
                constraint_penalty += penalization


        elif self.own_device_state == "FuelCell":
            """    
            - 0 <= P(i, t) <= p_max -> via bounds 
            """
            F0 = self.device.state.fuel_amount #F(i, 0) = fuel_amount
            Ft = F0
            P1t = self.device.state.p_prev
            for t, Pt in enumerate(schedule):
                Ft1 = Ft - Pt  #F(i, t + 1) = F(i, t) - P(i, t)
                if Ft < 0 or Ft > self.device.state.fuel_amount: #F(i, t) >= 0 and <= fuel amount
                    constraint_penalty += penalization
                if abs(Pt - P1t) > self.device.state.change_max: #first: | P(i, 0) - p_prev | <= change_max; then ramp: | P(i, t + 1) - P(i, t) | <= change_max
                    constraint_penalty += penalization
                Ft = Ft1
                P1t = Pt

        return constraint_penalty

    def helper_PSO_cost_fcn(self, schedules):
        P_tot = [sum(schedules[aid][0][t] for aid in schedules) for t in range(len(self.target))]
        P_dev = [abs(self.target[t]- P_tot[t]) for t in range(len(self.target))]
        sum_c_dev = sum(P_dev)*self.c_dev
        sum_c_op = sum(sum(abs(P) * self.dev_c_op[aid] for P in schedules[aid][0]) for aid in schedules)
        c_total = sum_c_dev + sum_c_op
        penalty = self.constraint_cost(schedules)
        #print(self.aid, f"c_total: {c_total}, penalty: {penalty}")
        return c_total+penalty

    def PSO_cost_fcn(self, particles):
        costs = []
        for particle in particles:
            schedules = self.GB_schedules
            schedules[self.aid] = [particle.tolist()] #replacing own schedule with current position in schedules
            cost = self.helper_PSO_cost_fcn(schedules) #calculating cost with helper_cost_fcn
            costs.append(cost)
        return np.array(costs)

    def add_constraints(self):
        if self.own_device_state == "Load":
            p_min_bounds = np.full(len(self.target), self.device.state.p_min)
            p_max_bounds = np.full(len(self.target),self.device.state.p_max)
            bounds = (p_min_bounds, p_max_bounds)
        elif self.own_device_state == "Battery":
            p_min_bounds = np.full(len(self.target), self.device.state.p_min)
            p_max_bounds = np.full(len(self.target), self.device.state.p_max)
            bounds = (p_min_bounds, p_max_bounds)
        elif self.own_device_state == "FuelCell":
            p_min_bounds = np.full(len(self.target), self.device.state.p_min)
            p_max_bounds = np.full(len(self.target), self.device.state.p_max)
            bounds = (p_min_bounds, p_max_bounds)
        return bounds

    async def solve_PSO_decentral(self):
        print("starting PSO solving")

        self.PSO_counter +=1
        bounds = self.add_constraints()
        n_particles = 1000
        n_dimensions = len(self.target)
        hyp_param = {'c1': 1.7, #cognitiv parameter = how much each particle is influenced by own PB
                     'c2': 1.5, #social parameter = weights how much particle is influences by GB
                     'w': 0.1} # inertia parameter = particles precious velocity
        PSO_optimizer = ps.single.GlobalBestPSO(n_particles = n_particles,
                                                dimensions = n_dimensions,
                                                options = hyp_param,
                                                bounds = bounds)
        start_time = time.perf_counter()
        sugg_GB_cost, best_pos = PSO_optimizer.optimize(self.PSO_cost_fcn, iters = 200)
        end_time = time.perf_counter()
        solver_runtime = end_time - start_time
        print(f"{self.aid} PSO solver runtime (iteration {self.PSO_counter}): {solver_runtime:.4f} seconds")
        best_pos = best_pos.tolist()
        #print(self.aid, "best pos", best_pos)
        self.device_schedule = [best_pos]
        sugg_GB_scheduels = self.GB_schedules
        sugg_GB_scheduels[self.aid] = [best_pos]
        self.all_GB_schedules[self.aid] = [best_pos]
        self.all_GB_costs[self.aid] = sugg_GB_cost
        #print(self.aid, f"best cost {sugg_GB_cost} with best schedule: {sugg_GB_scheduels}")
        self.PSO_process.set_result(True)
        await self.evaluate_schedule(sugg_GB_scheduels)

    async def reschedule(self, remaining_target, timestep):
        print("here should be the rescheduling part ")
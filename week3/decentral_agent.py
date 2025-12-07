from mango import Agent, sender_addr
from src.sim_environment.optimization_problem import SchedulingProblem
from copy import deepcopy
from pyomo_solver import ED_solve, UC_solve
import asyncio

from messages import *
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
        self.committed = True
        self.commited_units = None
        self.device_schedule = [0] * len(target)

        # various futures objects for control flow
        self.init_schedule_done = asyncio.Future()
        self.done = asyncio.Future()
        self.state_request_fut = asyncio.Future()
        self.all_replies_arrived = None

        # for dumb testing
        self.device_replies = {}

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

    def on_register(self):
        pass

    def on_start(self):
        print(f"{self.aid} at: {self.addr}")
        print(f"{self.aid} neighbors: {self.neighbors()}")
        self.schedule_instant_task(self.identify_agents())
        self.schedule_instant_task(self.routing())
        #self.schedule_instant_task(self.create_initial_schedule())

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

        if isinstance(content, GetDeviceInformationMsg):
            self.schedule_instant_task(self.handle_GetDeviceInformationMsg(content, sender))

        if isinstance(content, ReplyDeviceInformationMsg):
            self.schedule_instant_task(self.handle_ReplyDeviceInformationMsg(content, sender))

        if isinstance(content, SendScheduleMsg):
            self.schedule_instant_task(self.handle_SendScheduleMsg(content, sender))

        if isinstance(content, UpdateDeviceInformationMsg):
            self.schedule_instant_task(self.handle_update_device_information(content, sender))

        if isinstance(content, ReplyUpdateDeviceInformationMsg):
            self.schedule_instant_task(self.handle_ReplyUpdateDeviceInformationMsg(content, sender))


    ### Helper function
    async def forward_msg(self, content, sender):
        other_neighbors = [n for n in self.neighbors() if n != sender]  # forward message to other neigbors
        if other_neighbors:
            for neighbor in other_neighbors:  # send message only to others, not to sender not notify
                print(self.aid, "forwarding msg to", neighbor)
                await self.send_message(content, neighbor)
        else:
            print(self.aid, "no other neighbors than parent, sending it to parent")
            await self.send_message(content, sender)  # start sending it back to sender


    ### Msg Handling
    async def handle_target_update(self, content, meta):
        if self.leader and self.target[content.t] != content.value:
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
                    print(self.aid, "registering to agents dict")
                    agents[self.aid] = {"parent": sender}  # add aid and parent to dict
                    self.registered_for[original_sender] = {"access_via": sender}  # writing sender_aid to registered_to list
                    await self.forward_msg(content, sender)
                else:
                    print(self.aid, "something went wrong, I do nto remember registering")

        elif original_sender in self.registered_for: #I already registered once to this agent and forwarded it afterwards
                print(self.aid, "sending msg to direction of original sender")
                parent = self.registered_for[original_sender]["access_via"]
                await self.send_message(content, parent)

        if content.sender_aid == self.aid:  # if I am original sender and don't forward the message anymore
            print(self.aid, "I got my message back")
            for aid, data in content.agents.items():  # if there are agents yet unkown in received list, append them
                if aid not in self.agent_info:
                    self.agent_info[aid] = data
            if len(self.agent_info) < len(self.registered_for):
                print("I registered to more than I received and wait a bit longer")
            print(self.aid, "the agents I know are:", self.agent_info)



    async def identify_agents(self):
        # send message to all agents to find all agents
        agents = {}
        msg = IdentifyAgentsMsg(self.aid, agents=agents) #sending msg with own aid and empty agents list
        for neighbor in self.neighbors():  # send message with highest id to all neighbors
            print(self.aid, "sending message to: ", neighbor)
            await self.send_message(msg, neighbor)


    async def handle_SendScheduleMsg(self, content, sender):
        print(self.aid, content)
        receiver = content.receiver
        route = content.route

        if receiver == self.aid:  # reply with ReplyDeviceInformationMsg
            self.device_schedule = content.schedule
            if not self.init_schedule_done.done():
                self.init_schedule_done.set_result(True)
            else:
                await self.update_device_schedule()  # send to observer update device message

        else:
            for neighbor in self.neighbors():
                if neighbor.aid in route and neighbor.aid != sender.aid:
                    await self.send_message(content, neighbor)


    # ------------------------------------
    # ------------------------------------
    '''
    async def routing(self):
        await asyncio.sleep(6)
        parents = {}
        agents = self.agent_info
        for key, value in agents.items():
            parents[key] = value.aid
            # build full paths for every con_
        full_paths = {}

        for node in parents.keys():
            path = []
            current = node

            # Follow parent chain until reaching leader
            while current != self.leader_aid:
                path.append(current)
                current = parents[current]  # go to parent aid

            # reverse to get leader → node path
            path.reverse()
            full_paths[node] = path
        self.agent_routing = full_paths
        print(self.agent_routing)
    '''
    async def update_device_schedule(self):
        # for updating my own schedule
        print(self.aid, "update device schedule with ", self.device_schedule)
        msg = SetScheduleMsg(self.device_schedule)
        self.schedule_instant_message(msg, self.device_address)
        print(self.aid, "done")

    async def get_device_state(self):
        self.state_request_fut = asyncio.Future()
        msg = StateRequestMsg()
        await self.send_message(msg, self.device_address)
        await self.state_request_fut

    async def get_device_information(self):
        for agent_aid in self.agent_routing.keys():  # go through all agent_aids and get aid and route to aid
            self.get_device_future = asyncio.Future()
            if agent_aid != self.aid:
                receiver = agent_aid  # target agent
                route = self.agent_routing[agent_aid]  # gives list with route to target agent
                msg = GetDeviceInformationMsg(receiver, route)
                for neighbor in self.neighbors():
                    if receiver == neighbor.aid or neighbor.aid in route:
                        print("send message to ", neighbor.aid, "because: ", receiver, "=", neighbor.aid,
                              "or neighbor in ", route)
                        await self.send_message(msg, neighbor)

            elif agent_aid == self.aid:
                if self.aid not in self.device_replies:  # adding myself if not done yet
                    await self.get_device_state()
                    agent_aid = self.aid
                    self.device_replies[agent_aid] = {"device": self.device}
                    self.get_device_future.set_result(True)
            await self.get_device_future

    async def update_device_information(self):
        for agent_aid in self.agent_routing.keys():  # go through all agent_aids and get aid and route to aid
            self.state_update_future = asyncio.Future()
            if agent_aid != self.aid:
                receiver = agent_aid  # target agent
                route = self.agent_routing[agent_aid]  # gives list with route to target agent
                msg = UpdateDeviceInformationMsg(receiver, route)
                for neighbor in self.neighbors():
                    if receiver == neighbor.aid or neighbor.aid in route:
                        print("send message to ", neighbor.aid, "because: ", receiver, "=", neighbor.aid,
                              "or neighbor in ", route)
                        await self.send_message(msg, neighbor)
            elif agent_aid == self.aid:
                if self.aid in self.device_replies:
                    await self.get_device_state()
                    self.device_replies[agent_aid]["device"].state = self.device.state
                    self.state_update_future.set_result(True)

            await self.state_update_future

    async def handle_update_device_information(self, content, sender):
        receiver = content.receiver
        route = content.route
        # check if message is for me
        if receiver == self.aid:  # reply with ReplyDeviceInformationMsg
            await self.get_device_state()
            state = self.device.state
            leader = self.leader_aid
            agent_aid = self.aid

            msg = ReplyUpdateDeviceInformationMsg(agent_aid, state, leader)
            await self.send_message(msg, self.parent)

        else:  # forward the message to neighbor in route
            for neighbor in self.neighbors():
                if neighbor.aid in route and neighbor.aid != sender.aid:
                    await self.send_message(content, neighbor)

    async def handle_ReplyUpdateDeviceInformationMsg(self, content, sender):
        """logic: I get a message with state, receiver
        If I am receiver and leader, then I need to update the state and the cost for the device in a dict
        If I am not receiver, then I forward message to self.parent further to direction of leader"""
        receiver = content.receiver
        agent_aid = str(content.agent_aid)

        # check if I am receiver and leader
        if receiver == self.aid and self.leader:
            # creating device_replies dict with agent aid and device inside
            if agent_aid in self.device_replies:
                self.device_replies[agent_aid]["device"].state = content.state
                self.state_update_future.set_result(True)
        else:
            await self.send_message(content, self.parent)  # forward message to direction of leader

    async def handle_GetDeviceInformationMsg(self, content, sender):
        '''
        logic: I get a message with receiver and route,
        now I check if I am receiver, then I reply with my state,
        if I am not the sender I send it to my neighbor who is in the route list '''
        receiver = content.receiver
        route = content.route
        # check if message is for me
        if receiver == self.aid:  # reply with ReplyDeviceInformationMsg
            await self.get_device_state()
            state = self.device.state
            c_op = self.device.c_op
            commitment_cost = self.device.commitment_cost
            leader = self.leader_aid
            agent_aid = self.aid
            msg = ReplyDeviceInformationMsg(agent_aid, state, c_op, commitment_cost, leader)
            await self.send_message(msg, self.parent)
        else:  # forward the message to neighbor in route
            for neighbor in self.neighbors():
                if neighbor.aid in route and neighbor.aid != sender.aid:
                    await self.send_message(content, neighbor)

    async def handle_ReplyDeviceInformationMsg(self, content, sender):
        """logic: I get a message with state, cost, receiver
        If I am receiver and leader, then I need to save the state and the cost for the device in a dict
        If I am not receiver, then I forward message to self.parent further to direction of leader"""
        receiver = content.receiver
        agent_aid = str(content.agent_aid)
        state = content.state
        c_op = content.c_op
        commitment_cost = content.commitment_cost
        # check if I am receiver and leader
        if receiver == self.aid and self.leader:
            # creating device_replies dict with agent aid and device inside
            if agent_aid not in self.device_replies:
                self.device_replies[agent_aid] = {"device": IdealDevice(state, c_op, commitment_cost)}
                self.get_device_future.set_result(True)

            if len(self.device_replies) == len(self.agent_routing):
                if not self.all_replies_arrived.done():
                    self.all_replies_arrived.set_result(True)
                    print(self.device_replies)
        else:
            await self.send_message(content, self.parent)  # forward message to direction of leader

    """
    Call your schedule solver here and save the initial schedule.
    """
    async def select_random_start(self, target_length):
        state = self.device.state
        print(self.aid, state)
    async def create_initial_schedule(self):
        await asyncio.sleep(6)
        target_length = len(self.target)
        await self.select_random_start(target_length)

        #Todo wait for all scheudles from other agents I know
        #Todo run ED solve for all initial random schedules & optimize only own one
        #Todo check if PB changes (initally inf) -> if yes save cost and schedules
        #Todo when GB changes (initially inf) -> if yes save cost and scheudle and compare with others (send msg and when msg received if new gloabl best != own gloabal best, set new and send to neighbors)
        #Todo Do PSO to get random deviations of others -> reoptimize own and check for change

        #self.commited_units = await self.solve_UC_decentral()
        self.all_device_schedules = await self.solve_ED_decentral(self.target)



    async def solve_UC_decentral(self):
        self.all_replies_arrived = asyncio.Future()
        await self.get_device_information()
        await self.all_replies_arrived
        self.all_devices = [entry["device"] for entry in self.device_replies.values()]
        print(self.all_devices)
        commited_units = UC_solve(self.all_devices, self.target, self.c_dev)
        print(commited_units)
        return commited_units

    async def solve_ED_decentral(self, target):
        print("starting ED solve")
        all_device_schedules, costs = ED_solve(self.all_devices, self.commited_units, target, self.c_dev)
        print(all_device_schedules)
        return all_device_schedules

    '''Implement your rescheduling logic for the agent here.'''

    async def reschedule(self, remaining_target, t):
        await self.update_device_information()
        new_schedules = await self.solve_ED_decentral(remaining_target)

        print("rescheduling", self.all_device_schedules)
        for i, agent_aid in enumerate(
                self.device_replies.keys()):  # go through all agent_aids and get aid and route to aid (same dict as devices)
            self.all_device_schedules[i][t:] = new_schedules[i][:]
            if agent_aid == self.aid:
                self.device_schedule = self.all_device_schedules[i]
                if not self.init_schedule_done.done():
                    self.init_schedule_done.set_result(True)
            elif agent_aid != self.aid:
                receiver = agent_aid  # target agent
                route = self.agent_routing[agent_aid]  # gives list with route to target agent
                msg = SendScheduleMsg(schedule=self.all_device_schedules[i], route=route, receiver=receiver)
                for neighbor in self.neighbors():
                    if receiver == neighbor.aid or neighbor.aid in route:
                        await self.send_message(msg, neighbor)

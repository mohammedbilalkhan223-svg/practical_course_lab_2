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
    StateReplyMsg,
    LeaderFoundMsg,
    FindLeaderMsg,
)

class AggregatingAgent(Agent):
    def __init__(self, device_address, device, target, c_dev):
        super().__init__()
        self.device_address = device_address #device address of the device agent is sitting on
        self.device = device #device object to access device information
        self.target = deepcopy(target)
        self.c_dev = c_dev

        # to be set accordingly during the initial scheduling!
        self.committed = True 
        self.device_schedule = [0] * len(target)

        # various futures objects for control flow
        self.init_schedule_done = asyncio.Future()
        self.done = asyncio.Future()
        self.state_request_fut = asyncio.Future()
        self.same_neighbor_amount = asyncio.Future()

        # for dumb testing
        self.counter = 0
        self.leader = False #set only True for leader
        self.leader_id = None
        self.leader_info_source = None
        self.agent_info = None
        self.agent_routing = None

        self.leader_approved = asyncio.Future()

    def on_register(self):
        pass

    def on_start(self):
        print(f"{self.aid} at: {self.addr}")
        print(f"{self.aid} neighbors: {self.neighbors()}")
        self.schedule_instant_task(self.initialize_leader_selection())
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

        if isinstance(content, FindLeaderMsg):
            self.leader_approved = asyncio.Future()
            self.schedule_instant_task(self.handle_leader_selection(content, sender))

        if isinstance(content, LeaderFoundMsg):
            self.schedule_instant_task(self.approve_leader_selection(content, sender))

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

    async def handle_leader_selection(self, content, sender):
        received_id = content.leader_id
        if received_id > self.leader_id or self.leader_id == None: #if received larger than saved leader_id
            self.leader = False
            self.leader_id = received_id
            self.leader_info_source = sender
            #for neighbor in (other for other in self.neighbors() if other != sender): #send message only to others, not to sender not notify
            for neighbor in self.neighbors():
                msg = FindLeaderMsg(self.leader_id)
                await self.send_message(msg, neighbor)

        elif received_id < self.leader_id:
            for neighbor in (other for other in self.neighbors() if other != sender): #send message only to others, not to sender not notify
                msg = FindLeaderMsg(self.leader_id)
                await self.send_message(msg, neighbor)

        elif received_id == self.leader_id and received_id != int(self.my_id.split("_")[1]):
            #print(self.my_id, "I also saved this leader, now I notify.", self.leader_info_source)
            msg = FindLeaderMsg(self.leader_id)
            await self.send_message(msg, self.leader_info_source)
        elif received_id == int(self.my_id.split("_")[1]) and received_id == self.leader_id:
            print(self.my_id, "I am the leader")
            self.leader = True
            agents = {self.aid: "leader",
                      }
            for neighbor in self.neighbors(): #notify everyone around me that I know higher id
                msg = LeaderFoundMsg(self.leader_id, agents)
                await self.send_message(msg, neighbor)
                #print(self.my_id, "I send a leader found message to all", neighbor.aid)

        '''
        elif received_id == self.leader_id and received_id != int(self.my_id.split("_")[1]):
            print(self.my_id, "I also saved this leader")
            agents= {"aid":[self.aid],
                    "source": self.leader_info_source,
                     }
            for neighbor in self.neighbors():
                msg = LeaderFoundMsg(self.leader_id, agents)
                await self.send_message(msg, neighbor)
        '''
    async def approve_leader_selection(self, content, sender):
        received_id = content.leader_id
        agents = content.agents
        if self.aid not in agents: #if I am not in agent list already:
            agents[self.aid] = self.leader_info_source
            #check if received id is my id and I am the leader
            if received_id == int(self.my_id.split("_")[1]) and received_id == self.leader_id:
                #print(self.my_id, "I am leader and I know it")
                self.leader = True
            # received id already saved as leader and is not my id
            elif received_id == self.leader_id and received_id != int(self.my_id.split("_")[1]):
                #print(self.my_id, "I know the leader and approve it")
                other_neighbors = [n for n in self.neighbors() if n != sender]
                msg = LeaderFoundMsg(self.leader_id, agents)
                if other_neighbors:
                    for neighbor in other_neighbors:# send message only to others, not to sender not notify
                        await self.send_message(msg, neighbor)
                else:
                    await self.send_message(msg, sender)

            elif received_id < self.leader_id or received_id < int(self.my_id.split("_")[1]):
                print(self.my_id, "I have a problem with the leader, my suggestion is", self.leader_id)
                for neighbor in self.neighbors(): #notify everyone around me that I know higher id
                    msg = FindLeaderMsg(self.leader_id)
                    await self.send_message(msg, neighbor)
        else:
            if received_id == int(self.my_id.split("_")[1]):
                print(self.my_id, "I am the leader and I stop now")
                print("Agent list: ", agents)
                self.leader = True
                self.agent_info = agents
                try:
                    self.leader_approved.set_result(True)
                    print("Leader approved")
                except:
                    print("future already set")

            else:
                print("my AID is in list but I am not leader")
                self.agent_info = agents
                other_neighbors = [n for n in self.neighbors() if n != sender]
                msg = LeaderFoundMsg(self.leader_id, agents)
                if other_neighbors:
                    for neighbor in other_neighbors:  # send message only to others, not to sender not notify
                        await self.send_message(msg, neighbor)
                else:
                    await self.send_message(msg, sender)

        '''
        if content.neighbor_amount > self.neighbor_amount:
            self.same_neighbor_amount = asyncio.Future()
            highest_neighbor_amount = content.neighbor_amount
            id_highest_neighbor_amount = content.agent_id
            for neighbor in self.neighbors():
                msg = FindLeaderMsg(highest_neighbor_amount, id_highest_neighbor_amount) #send message with highest amount of neighbors and the id of the agent with the id
                await self.send_message(msg, neighbor)
        elif content.neighbor_amount < self.neighbor_amount:
            self.same_neighbor_amount = asyncio.Future()
            highest_neighbor_amount = self.neighbor_amount
        else:
            self.same_neighbor_amount = True'''

    async def handle_leader_role(self):
        parents = {}
        agents = self.agent_info
        leader_aid = self.aid
        for key, value in agents.items():
            if value == "leader":
                continue
            parents[key] = value.aid  # AgentAddress has .aid

            # Now: build full paths for every con_X
        full_paths = {}

        for node in parents.keys():
            path = []
            current = node

            # Follow parent chain until reaching the leader
            while current != leader_aid:
                path.append(current)
                current = parents[current]  # go to parent aid

            # At the end, reverse to get leader → node path
            path.reverse()

            full_paths[node] = path
        self.agent_routing = full_paths
        print(self.agent_routing)

    #------------------------------------
    #------------------------------------
    async def initialize_leader_selection(self):
        #idea: find agent with highest id
        self.my_id = self.aid
        self.leader_id = int(self.my_id.split("_")[1])
        for neighbor in self.neighbors():
            if self.leader_id > int(neighbor.aid.split("_")[1]): #check if my id is higher than of my neighbors
                self.leader = True
            else:
                #self.leader_id = int(neighbor.aid.split("_")[1])
                self.leader = False
        if self.leader: #if I am the leader send a message to the neighbors with my id number to compare with others
            print("I think I am the leader and now I send a message", self.my_id)
            msg = FindLeaderMsg(self.leader_id)
            for neighbor in self.neighbors(): #send message with highest id to all neighbors
                await self.send_message(msg, neighbor)
        '''
        if not self.same_neighbor_amount:
            current_best_leader = None
            for neighbor in self.neighbors():
                print("neighbor id", neighbor.aid)
                msg = FindLeaderMsg(self.neighbor_amount, neighbor.aid)
                await self.send_message(msg, neighbor)'''




    async def update_device_schedule(self):
        msg = SetScheduleMsg(self.device_schedule)
        self.schedule_instant_message(msg, self.device_address)
        

    async def get_device_state(self):
        self.state_request_fut = asyncio.Future()   
        msg = StateRequestMsg()
        await self.send_message(msg, self.device_address)
        await self.state_request_fut

    async def get_device_information(self):
        print(self.aid, "now I need to get the device information")
        await self.get_device_state()
    """
    Call your schedule solver here and save the initial schedule.
    """
    async def create_initial_schedule(self):
        await asyncio.sleep(5)
        if self.leader == True:
            await self.handle_leader_role()
            #await self.get_device_information()
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

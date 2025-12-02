from dataclasses import dataclass
from mango.messages.codecs import json_serializable, JSON
from typing import Union
from src.sim_environment.devices.abstract import AbstractState

from src.sim_environment.devices.ideal import *
from src.sim_environment.devices.hil import IdealBatteryState as HILBatteryState
from src.sim_environment.devices.hil import IdealLoadState as HILLoadState
from src.sim_environment.devices.hil import IdealFuelCellState as HILFuelState


SCENARIO_CODEC = JSON()
# state types
SCENARIO_CODEC.add_serializer(*IdealBatteryState.__serializer__())
SCENARIO_CODEC.add_serializer(*IdealLoadState.__serializer__())
SCENARIO_CODEC.add_serializer(*IdealFuelCellState.__serializer__())

SCENARIO_CODEC.add_serializer(*HILBatteryState.__serializer__())
SCENARIO_CODEC.add_serializer(*HILLoadState.__serializer__())
SCENARIO_CODEC.add_serializer(*HILFuelState.__serializer__())


# predefined msg types
#------------------------------
# Both
#------------------------------
@json_serializable
@dataclass
class SetDoneMsg:
    pass

SCENARIO_CODEC.add_serializer(*SetDoneMsg.__serializer__())

#------------------------------
# Device Agent
#------------------------------
@json_serializable
@dataclass
class StateRequestMsg:
    pass

SCENARIO_CODEC.add_serializer(*StateRequestMsg.__serializer__())


@json_serializable
@dataclass
class StateReplyMsg:
    state: AbstractState

SCENARIO_CODEC.add_serializer(*StateReplyMsg.__serializer__())


@json_serializable
@dataclass
class SetScheduleMsg:
    setpoints: list[float]

SCENARIO_CODEC.add_serializer(*SetScheduleMsg.__serializer__())


@json_serializable
@dataclass
class SetScheduleReplyMsg:
    ok: bool

SCENARIO_CODEC.add_serializer(*SetScheduleReplyMsg.__serializer__())

@json_serializable
@dataclass
class StepMsg:
    t: int

SCENARIO_CODEC.add_serializer(*StepMsg.__serializer__())

@json_serializable
@dataclass
class StepReplyMsg:
    t: int

SCENARIO_CODEC.add_serializer(*StepReplyMsg.__serializer__())

@json_serializable
@dataclass
class StepReplyMsg:
    state: object
    costs: list[float]

SCENARIO_CODEC.add_serializer(*StepReplyMsg.__serializer__())

@json_serializable
@dataclass
class ResultLogRequestMsg:
    pass

SCENARIO_CODEC.add_serializer(*ResultLogRequestMsg.__serializer__())

@json_serializable
@dataclass
class ResultLogMsg:
    cumulative_cost: float
    power_setpoints: list[float]

SCENARIO_CODEC.add_serializer(*ResultLogMsg.__serializer__())

#------------------------------
# Control Agent
#------------------------------
# predefined msg types
@json_serializable
@dataclass
class TargetUpdateMsg:
    t: int
    value: float

SCENARIO_CODEC.add_serializer(*TargetUpdateMsg.__serializer__())


# predefined msg types
@json_serializable
@dataclass
class NotifyReadyRequestMsg:
    pass

SCENARIO_CODEC.add_serializer(*NotifyReadyRequestMsg.__serializer__())

# predefined msg types
@json_serializable
@dataclass
class NotifyReadyMsg:
    pass

SCENARIO_CODEC.add_serializer(*NotifyReadyMsg.__serializer__())

#------------------------------
# Aggregating Agent
#------------------------------
@json_serializable
@dataclass
class FindLeaderMsg:
    leader_id: int

SCENARIO_CODEC.add_serializer(*FindLeaderMsg.__serializer__())


@json_serializable
@dataclass
class LeaderFoundMsg:
    leader_id: int
    agents: dict
    leader_aid: str


SCENARIO_CODEC.add_serializer(*LeaderFoundMsg.__serializer__())

@json_serializable
@dataclass
class GetDeviceStateMsg:
    receiver: str
    route: int

SCENARIO_CODEC.add_serializer(*GetDeviceStateMsg.__serializer__())

@json_serializable
@dataclass
class ReplyDeviceStateMsg:
    agent_aid : str
    state: AbstractState
    c_op: float
    commitment_cost: float
    receiver: str

SCENARIO_CODEC.add_serializer(*ReplyDeviceStateMsg.__serializer__())
from __future__ import annotations
import os
import sys


current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)
sys.path.append(os.path.normpath(os.path.join(current_path, '../')))
sys.path.append(os.path.normpath(os.path.join(current_path, '../../')))

from enum import Enum, IntEnum, auto
from math import atan2, pow, sqrt

class ControllerFrom(Enum):
    BuiltIn = auto()
    External = auto()
    MoraiSim_Drive = auto()

class NpcStatus(Enum):
    Idle = 1            # Before Initialization, or After completion (the controller would be deleted)
    Default = 2         # Lane following
    Action = 3          # Scenario action is activated
    Completed = 4       # Scenario action is completed


class Position:
    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other: Position) -> Position:
        return Position(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Position) -> Position:
        return Position(self.x - other.x, self.y - other.y, self.z - other.z)

    def __str__(self):
        return str(self.__dict__)

    def distance(self, other: Position) -> float:
        return sqrt(pow(self.x - other.x, 2) + pow(self.y - other.y, 2))

    def angle(self, other: Position) -> float:
        return atan2(other.y - self.y, other.x - self.x)


class AccObject:
    def __init__(
        self,
        idx: str,
        distance: float,
        velocity: float,
        ego_front_length: float,
        object_rear_length: float,
        margin_distance: float = 3
    ):
        self._idx = idx
        self._distance = distance
        self._velocity = velocity
        # TODO: ego length를 더 쉽게 넣는 방법 고민하기.
        self._ego_front_length = ego_front_length
        self._object_rear_length = object_rear_length
        self._margin_distance = margin_distance
    
    def __str__(self):
        return str(self.__dict__)

    def __eq__(self, other):
       return self.distance == other.distance
    
    def __lt__(self, other):
        return self.distance < other.distance

    @property
    def idx(self):
        return self._idx

    @property
    def distance(self):
        return self._distance - self._ego_front_length - self._object_rear_length

    @property
    def velocity(self):
        return self._velocity
    
    @property
    def margin_distance(self):
        return self._margin_distance

    def is_in_dangerous_zone(self) -> bool:
        return round(self.distance - self.margin_distance, 1) <= 0


class ControlCommand:
    def __init__(self, unique_id:str=None):
        self._unique_id: str = unique_id
        
        # control command
        self._longCmdType: int = 0
        self._accel: float = 0
        self._brake: float = 0
        self._velocity: float = 0
        self._acceleration: float = 0
        self._steering: float = 0
        
        # event command
        self._option: int = 2
        self._ctrl_mode: int = 0
        self._gear: int = GearChange.NONE
        self._turn_signal: int = TurnSignal.UNSPECIFIED
        self._emergency_signal: int = 0
        self._set_pause: bool = False

    @property
    def unique_id(self) -> str:
        if self._unique_id is None:
            raise ValueError("Unique id must be initialized before usage")
        return self._unique_id
    @unique_id.setter
    def unique_id(self, id: str):
        self._unique_id = id
    
    @property
    def longCmdType(self) -> int:
        return self._longCmdType

    @property
    def accel(self) -> float:
        return self._accel
    @accel.setter
    def accel(self, value: float):
        self._longCmdType = 1
        self._accel = value
        self._brake = 0
    
    @property
    def brake(self) -> float:
        return self._brake
    @brake.setter
    def brake(self, value: float):
        self._longCmdType = 1
        self._accel = 0
        self._brake = value

    @property
    def velocity(self) -> float:
        return self._velocity
    @velocity.setter
    def velocity(self, value: float):
        self._longCmdType = 2
        self._velocity = value
    
    @property
    def acceleration(self) -> float:
        return self._acceleration
    @acceleration.setter
    def acceleration(self, value):
        self._longCmdType = 3
        self._acceleration = value

    @property
    def steering(self) -> float:
        return self._steering
    @steering.setter
    def steering(self, value: float):
        self._steering = value
    
    @property
    def option(self) -> int:
        return self._option
    @option.setter
    def option(self, value: int):
        self._option = value
    
    @property
    def ctrl_mode(self) -> int:
        return self._ctrl_mode
    @ctrl_mode.setter
    def ctrl_mode(self, value: int):
        self._ctrl_mode = value

    @property
    def gear(self) -> int:
        return self._gear
    @gear.setter
    def gear(self, value: GearChange):
        self._gear = value
    
    @property
    def turn_signal(self) -> int:
        return self._turn_signal
    @turn_signal.setter
    def turn_signal(self, value: TurnSignal):
        self._turn_signal = value
    
    @property
    def emergency_signal(self) -> int:
        return self._emergency_signal
    @emergency_signal.setter
    def emergency_signal(self, value: int):
        self._emergency_signal = value
    
    @property
    def set_pause(self) -> bool:
        return self._set_pause
    @set_pause.setter
    def set_pause(self, value: bool):
        self._set_pause = value
    
    def set_full_brake(self):
        self.brake = 1

    def set_idle_command(self):
        self.set_full_brake()
        self.steering = 0.0

    @staticmethod
    def get_reset_light_request(unique_id) -> ControlCommand:
        command = ControlCommand(unique_id)
        command.option = 4
        command.turn_signal = TurnSignal.UNSPECIFIED
        command.emergency_signal = 0
        return command

class GearChange(IntEnum):
    NONE = -1       # Not changing
    PARK = 1
    REVERSE = 2
    NEUTRAL = 3
    DRIVE = 4

class TurnSignal(IntEnum):
    UNSPECIFIED = 0
    LEFT = 1
    RIGHT = 2
    ALL = 3
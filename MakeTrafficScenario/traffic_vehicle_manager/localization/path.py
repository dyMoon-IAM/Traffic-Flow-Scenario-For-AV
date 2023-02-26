from __future__ import annotations

from enum import Enum
from typing import List

import numpy as np

from ..mgeo.lib.mgeo.class_defs.line import Line
from .defs import Position

class AfterCompletion(Enum):
    random = "random"
    hide = "hide"
    stop = "stop"

class Path:
    def __init__(
        self,
        positions: List[Position] = [],
        link_idx_list: List[str] = [],
        last_link: Line = None,
        after_completion: AfterCompletion = AfterCompletion.random,
        start_waypoint_idx: int = 0,
        end_waypoint_idx: int = -1,
        is_link_based_path: bool = True,
    ):
        self._positions = positions
        self._link_idx_list = link_idx_list
        # TODO: 외부에서 입력받는 게 아니라, link_idx_list 가 들어오면 자동으로 찾게 하고 싶지만, link_idx 만 받아 어려움.
        self.last_link = last_link
        self.after_completion = after_completion
        self.start_waypoint_idx = start_waypoint_idx
        self.end_waypoint_idx = end_waypoint_idx

        # TODO: Link 에 기반하여 path system 이 구성되어 있어, 이러한 property 가 필요하며, 리팩토링을 하여 개선할 필요가 있음.
        self.is_link_based_path = is_link_based_path

        # TODO: 다른 방식으로 path update 의 trigger 가 되도록 개선해야 함.
        self.is_changed_path = True

        # TODO: 이렇게 route action 여부를 확인 하는 것이 맞는지 고민하기.
        self.is_route_action = True

    @property
    def positions(self) -> List[Position]:
        return self._positions

    # TODO: positions 를 생성한 이후 add 가 아닌 set 으로 변경하는 것에 대한 적절한 고민 필요.
    #  link_idx_list 도 영향을 받으며, positions 가 바뀌는 경우는 새로운 Path 인스턴스를 만들어야 하는 게 아닌가 생각도 듬.
    @positions.setter
    def positions(self, value: List[Position]):
        self._positions = value

    @property
    def link_idx_list(self) -> List:
        return self._link_idx_list

    @property
    def last_link(self):
        return self._last_link

    @last_link.setter
    def last_link(self, value: Line):
        self._last_link = value

    def is_finished_path(self) -> bool:
        if self._last_link:
            return len(self._last_link.to_node.to_links) == 0
        else:
            return False

    def get_length(self):
        return len(self.positions)

    def is_empty(self):
        return self.get_length() == 0

    # TODO: 해당 함수 사용처가 현재 없으며, 확인 후 필요없으면 제거하기
    def calc_remain_dist_to_dest(self, current_waypoint: int):
        dist = 0.0
        for point0, point1 in zip(self.positions[current_waypoint:], self.positions[current_waypoint+1:]):
            dist += point0.distance(point1)
        return dist

    def get_vehicle_route_link(self):
        route_link = []
        for i, link_idx in enumerate(self.link_idx_list):
            link = {"unique_id" : link_idx , "index" : -1}
            if i == len(self.link_idx_list) -1:
                link["index"] = self.end_waypoint_idx
            elif i == 0:
                link["index"] = self.start_waypoint_idx
            route_link.append(link)

        return route_link

    def find_current_waypoint(self, vehicle_position: Position) -> int:
        distance_list = [vehicle_position.distance(position) for position in self.positions]
        return np.argmin(distance_list)

    def add_path(self, other: Path, other_start_idx: int = 0):
        self._positions = self.positions + other.positions[other_start_idx:]
        self._link_idx_list = self.link_idx_list + other.link_idx_list
        self.end_waypoint_idx = other.end_waypoint_idx
        self.is_route_action = other.is_route_action
        if other.last_link:
            self.last_link = other.last_link

    def get_local_path(self, vehicle_position: Position, local_path_length: int = 100):
        local_path = Path()
        current_waypoint = 0
        num_point = self.get_length()
        # TODO: Path 에서 positions 와 link 가 매칭이 안 되다보니까,
        #  경로가 겹쳐져서 그려지게 될 경우 제대로 처리할 수 없어 이에 대한 개선 필요.
        if num_point > 1:
            # sypark: Assumes constant interval
            interval = self.positions[1].distance(self.positions[0])
            min_distance = float('inf')
            idx = 0
            while idx < num_point:
                distance = vehicle_position.distance(self.positions[idx])
                if distance < min_distance:
                    min_distance = distance
                    current_waypoint = idx
                idx += max(1, int(distance/interval))

        last_local_waypoint = min(num_point, current_waypoint + local_path_length)
        local_path.positions = self.positions[current_waypoint:last_local_waypoint]

        return local_path, current_waypoint

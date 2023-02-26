#!/usr/bin/env python3

### 
import pandas as pd

import json
import io
import os
import sys

from ..localization.point import Point
from .lib.mgeo.class_defs import *
from .e_dijkstra import Dijkstra
from ..localization.path import Path
from ..localization.defs import Position

from enum import Enum
import random
import itertools
class JunctionDirection(Enum):
    NONE = "none"
    RANDOM = "random"
    LEFT = "left"
    RIGHT = "right"
    STRAIGHT = "straight"
class mgeo_random_path:
    def __init__(self, map_name):

        current_path = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(current_path)

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/'+map_name))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines

    def plan_random_path(self, current_link_idx, route_direction=JunctionDirection.RANDOM):

        link_idx_list = [current_link_idx]
        while True:
            current_link: Link = self.links[current_link_idx]
            current_to_links = [link for link in current_link.to_node.to_links if not link.is_it_for_lane_change()]
            if len(current_to_links) == 1:
                current_link_idx = current_to_links[0].idx
            elif len(current_to_links) > 1:
                current_link_idx = self.select_to_link(current_to_links, route_direction)
            else:
                break

            # TODO: Link List 안에 중복된 Link 가 있는지 판단 로직, 경로 생성 종료 기준 으로 사용 되고 있어 추후 고도화 및 필요성 고민 필요
            if current_link_idx not in link_idx_list:
                link_idx_list.append(current_link_idx)
            else:
                break

        random_path = self.extract_path_from_link_idx_list(link_idx_list)
        # random_path.is_route_action = False
        return random_path

    # def select_to_link(self, to_links, current_link, junction_direction: JunctionDirection):
    def select_to_link(self, to_links, junction_direction: JunctionDirection):
        if junction_direction == JunctionDirection.RANDOM:
            selected_link_idx = random.choice(to_links).idx
        # elif junction_direction == JunctionDirection.LEFT:
        #     to_link_angle_dict = self.calculate_links_angle_difference(to_links, current_link)
        #     selected_link_idx = max(to_link_angle_dict, key=to_link_angle_dict.get)
        # elif junction_direction == JunctionDirection.RIGHT:
        #     to_link_angle_dict = self.calculate_links_angle_difference(to_links, current_link)
        #     selected_link_idx = min(to_link_angle_dict, key=to_link_angle_dict.get)
        # elif junction_direction == JunctionDirection.STRAIGHT:
        #     to_link_angle_dict = self.calculate_links_angle_difference(to_links, current_link)
        #     # straight 는 각도 차이가 제일 안 나는 link 로 설정
        #     selected_link_idx = min(to_link_angle_dict, key=lambda y: abs(to_link_angle_dict[y]))
        else:
            selected_link_idx = to_links[0].idx
        return selected_link_idx

    def extract_path_from_link_idx_list(self, link_idx_list):
        link_path = Path(
            positions=self.__extract_positions_from_link_idx_list(link_idx_list),
            link_idx_list=link_idx_list,
            last_link=self.links[link_idx_list[-1]]
        )
        x_list = []
        y_list = []
        z_list = []
        for i,position in enumerate(link_path.positions):
            x_list.append(position.x)
            y_list.append(position.y)
            z_list.append(position.z)


        path = pd.DataFrame({"x": x_list, "y": y_list, "z": z_list})
        path = path.apply(
            lambda point: Point(point["x"], point["y"]), axis=1
        ).tolist()
        return path

    def __extract_positions_from_link_idx_list(self, link_idx_list):
        positions_list = [self.__extract_positions_from_link_idx(link_idx) for link_idx in link_idx_list]
        positions = list(itertools.chain.from_iterable(positions_list))
        return positions
    def __extract_positions_from_link_idx(self, link_idx):
        positions = [self.__convert_to_position(point) for point in self.links[link_idx].points]
        return positions
    @staticmethod
    def __convert_to_position(point):
        return Position(*point)

class mgeo_dijkstra_path:

    def __init__(self, map_name):

        current_path = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(current_path)

        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/'+map_name))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        node_set = mgeo_planner_map.node_set
        link_set = mgeo_planner_map.link_set
        self.nodes=node_set.nodes
        self.links=link_set.lines

        self.global_planner=Dijkstra(self.nodes,self.links)

    def calc_dijkstra_path(self, start_node, end_node):

        result, path = self.global_planner.find_shortest_path(start_node, end_node)
        self.x_list = []
        self.y_list = []
        self.z_list = []
        for waypoint in path["point_path"] :
            path_x = waypoint[0]
            path_y = waypoint[1]
            self.x_list.append(path_x)
            self.y_list.append(path_y)
            self.z_list.append(0.)

        path = pd.DataFrame({"x":self.x_list, "y":self.y_list, "z":self.z_list})

        path_data = path.apply(
            lambda point: Point(point["x"], point["y"]), axis=1
        ).tolist()

        return path_data